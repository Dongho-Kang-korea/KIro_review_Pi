"""arbiter의 최종 속도 명령을 좌우 모터와 롤러 CAN 명령으로 변환한다."""

import json
import signal
import struct
import sys
import time

import can

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

class MotorController(Node):
    def __init__(self):
        super().__init__('motor')

        # 모터 ID와 속도 제한은 base/config/motor.yaml에서 조정한다.
        self.declare_parameter('test_single_id', 0)  # 0은 좌우 구동, 양수는 해당 모터만 시험
        self.declare_parameter('left_id', 1)
        self.declare_parameter('right_id', 2)
        self.declare_parameter('roller_id', 3)
        self.declare_parameter('vel_scale', 22.5)
        self.declare_parameter('max_vel', 45.0)
        self.declare_parameter('invert', True)
        self.declare_parameter('max_accel', 7.5)  # rev/s^2
        self.declare_parameter('max_decel', 15.0)  # rev/s^2
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('keepalive_interval', 0.05)  # CAN 감시 유지 주기
        # 롤러 속도 제한은 주행 모터와 별도로 조정한다.
        self.declare_parameter('roller_scale', 5.0)
        self.declare_parameter('roller_max_vel', 10.0)
        self.declare_parameter('roller_invert', False)
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('require_can', False)
        # 어댑터가 빠졌다 붙을 때 다시 여는 간격(초).
        self.declare_parameter('can_reconnect_interval_s', 2.0)
        self.declare_parameter('can_tx_interval_s', 0.02)
        # 0x14(전류)는 컨트롤러가 주기 송신하지 않아 RTR 조회가 유일한
        # 경로다(2026-08-31 실측). 0x09(엔코더)는 안 물어봐도 100Hz로 온다.
        self.declare_parameter('enable_rtr_requests', True)
        self.declare_parameter('current_request_rate', 10.0)
        self.declare_parameter('can_response_timeout_s', 0.5)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)
        self.declare_parameter('roller_cmd_timeout_s', 0.5)
        self.declare_parameter('contact_current_threshold_a', 0.0)
        self.test_id = self.get_parameter('test_single_id').value
        self.left_id = self.get_parameter('left_id').value
        self.right_id = self.get_parameter('right_id').value
        self.roller_id = self.get_parameter('roller_id').value
        self.vel_scale = self.get_parameter('vel_scale').value
        self.max_vel = self.get_parameter('max_vel').value
        self.invert = self.get_parameter('invert').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_decel = self.get_parameter('max_decel').value
        self.control_rate = self.get_parameter('control_rate').value
        self.keepalive_interval = self.get_parameter('keepalive_interval').value
        self.roller_scale = self.get_parameter('roller_scale').value
        self.roller_max_vel = self.get_parameter('roller_max_vel').value
        self.roller_invert = self.get_parameter('roller_invert').value
        self.can_channel = str(self.get_parameter('can_channel').value)
        self.require_can = bool(self.get_parameter('require_can').value)
        self.can_reconnect_interval = max(0.5, float(
            self.get_parameter('can_reconnect_interval_s').value))
        self.can_tx_interval = max(
            0.0, float(self.get_parameter('can_tx_interval_s').value))
        self.enable_rtr_requests = bool(
            self.get_parameter('enable_rtr_requests').value)
        self.current_request_rate = max(
            1.0, float(self.get_parameter('current_request_rate').value))
        self.contact_current_threshold_a = float(
            self.get_parameter('contact_current_threshold_a').value)
        self.can_response_timeout = float(
            self.get_parameter('can_response_timeout_s').value)
        self.cmd_vel_timeout = float(
            self.get_parameter('cmd_vel_timeout_s').value)
        self.roller_timeout = float(
            self.get_parameter('roller_cmd_timeout_s').value)
        self.last_roller_time = time.monotonic()
        self.sign = -1.0 if self.invert else 1.0

        # PCAN-USB 어댑터가 접촉 불량으로 탈부착되면(dmesg: 'Rx urb aborted',
        # 'can0 removed') 소켓이 죽어 recv/send 가 [Errno 19] No such device 로
        # 예외를 던진다. 예전에는 이 예외가 그대로 올라가 노드가 종료돼서,
        # 어댑터가 다시 붙어도 모터 제어가 영영 안 돌아왔다(2026-09-01 실측).
        # 이제는 버스를 놓고 주기적으로 다시 여는 것으로 스스로 복구한다.
        self._can_reconnect_at = 0.0
        self._can_lost_logged = False
        self._open_bus(initial=True)

        if self.test_id > 0:
            self.drive_motor_ids = [self.test_id]
            self.motor_ids = [self.test_id, self.roller_id]
            self.get_logger().info(f'*** 단일모터 테스트 모드: motor ID={self.test_id} 만 구동 ***')
        else:
            self.drive_motor_ids = [self.left_id, self.right_id]
            self.motor_ids = [self.left_id, self.right_id, self.roller_id]
            self.get_logger().info(
                f'주행 모터: LEFT ID={self.left_id}, RIGHT ID={self.right_id} | '
                f'독립 롤러 ID={self.roller_id}')

        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_vels = {i: 0.0 for i in self.motor_ids}
        self.applied_vels = {i: 0.0 for i in self.motor_ids}
        self.axis_status = {
            i: {'axis_error': None, 'axis_state': None,
                'position': None, 'encoder_velocity': None,
                'encoder_time': None,
                'iq_setpoint': None, 'current_a': None,
                'current_time': None} for i in self.motor_ids
        }
        self.last_can_response_time = None
        self.current_seq = 0
        # 실제 전송값을 저장해 작은 변화의 반복 전송을 막는다.
        self._sent = {}
        self._sent_time = {}
        self._can_backoff_until = 0.0
        self._last_can_tx_time = 0.0
        self._last_can_error_log = 0.0
        self._logged_cmd = None
        self.roller_torque_enabled = False
        self.declare_parameter('send_threshold', 0.02)  # CAN 재전송 최소 변화량
        self.send_threshold = self.get_parameter('send_threshold').value

        self.initialized = self.initialize_motors()

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.roller_subscription = self.create_subscription(
            Float32, 'roller_cmd', self.roller_cmd_callback, 10)

        self.status_pub = self.create_publisher(String, 'motor/status', 10)

        # 명령 수신과 CAN 출력을 분리해 가감속 제한을 일정하게 적용한다.
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self._control_tick)
        self.can_rx_timer = self.create_timer(0.02, self._read_can_status)
        self.encoder_timer = None
        self.current_timer = None
        if self.enable_rtr_requests:
            self.encoder_timer = self.create_timer(
                0.10, self._request_encoder_estimates)
            self.current_timer = self.create_timer(
                1.0 / self.current_request_rate, self._request_motor_currents)
            self.get_logger().warning(
                f'CAN RTR 조회 켜짐 — 전류(0x14) '
                f'{self.current_request_rate:.1f}Hz 감시')
        self.status_timer = self.create_timer(0.05, self._publish_status)

        self.log_timer = self.create_timer(1.0, self._log_status)

        self.get_logger().info('Motor Controller Started - Listening to /cmd_vel')

    def initialize_motors(self):
        if self.bus is None:
            return False
        # Closed loop보다 먼저 좌우 축의 목표를 명시적으로 0으로 만든다.
        for node_id in self.drive_motor_ids:
            if not self.set_velocity(node_id, 0.0):
                self.get_logger().error(
                    f'Motor initialization stopped: ID={node_id} zero send failed')
                return False
        for node_id in self.motor_ids:
            if not self.clear_errors(node_id):
                self.get_logger().error(
                    f'Motor initialization stopped: ID={node_id} clear error send failed')
                return False
            if not self.set_controller_velocity(node_id):
                self.get_logger().error(
                    f'Motor initialization stopped: ID={node_id} mode send failed')
                return False
            if node_id == self.roller_id:
                # 롤러는 첫 명령 전까지 토크를 해제한다.
                if not self.set_idle(node_id):
                    self.get_logger().error(
                        f'Motor initialization stopped: ID={node_id} idle send failed')
                    return False
            else:
                if not self.set_closed_loop(node_id):
                    self.get_logger().error(
                        f'Motor initialization stopped: ID={node_id} closed-loop send failed')
                    return False
        self.get_logger().info(f'Motors initialized: IDs={self.motor_ids}')
        return True

    def send_frame(self, node_id, cmd_id, data=b''):
        if self.bus is None:
            return False
        now = time.monotonic()
        if now < self._can_backoff_until:
            return False
        # PCAN-USB의 짧은 TX queue에 초기화 프레임이 한꺼번에 몰리지 않게 한다.
        remaining = self.can_tx_interval - (now - self._last_can_tx_time)
        if remaining > 0.0:
            time.sleep(remaining)
        arb_id = (node_id << 5) | cmd_id
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            self._last_can_tx_time = time.monotonic()
            return True
        except Exception as e:
            self._can_backoff_until = now + 1.0
            if now - self._last_can_error_log >= 5.0:
                self.get_logger().warning(
                    f'CAN transmit unavailable; retrying with backoff: {e}')
                self._last_can_error_log = now
            return False

    def request_frame(self, node_id, cmd_id):
        """어제 전류 실측에 성공한 형식 그대로 CANSimple RTR을 보낸다."""
        if self.bus is None:
            return
        try:
            self.bus.send(can.Message(
                arbitration_id=(node_id << 5) | cmd_id,
                is_extended_id=False,
                is_remote_frame=True))
        except Exception as e:
            now = time.monotonic()
            if now - self._last_can_error_log >= 5.0:
                self.get_logger().warning(
                    f'CAN RTR request failed (ID={node_id}, cmd=0x{cmd_id:02X}): {e}')
                self._last_can_error_log = now

    def _request_encoder_estimates(self):
        for node_id in self.drive_motor_ids:
            self.request_frame(node_id, 0x09)

    def _request_motor_currents(self):
        """Request ODrive CANSimple Get_Iq (0x14) for contact detection."""
        for node_id in self.drive_motor_ids:
            self.request_frame(node_id, 0x14)

    def clear_errors(self, node_id):
        return self.send_frame(node_id, 0x18, b'')

    def set_controller_velocity(self, node_id):
        return self.send_frame(node_id, 0x0B, struct.pack('<II', 2, 1))

    def set_closed_loop(self, node_id):
        return self.send_frame(node_id, 0x07, struct.pack('<I', 8))

    def set_idle(self, node_id):
        return self.send_frame(node_id, 0x07, struct.pack('<I', 1))

    def set_velocity(self, node_id, velocity):
        return self.send_frame(
            node_id, 0x0D, struct.pack('<ff', float(velocity), 0.0))

    def send_velocity_if_changed(self, node_id, velocity):
        """속도가 임계치 이상 바뀌었을 때만 CAN 전송.

        ODrive/Steadywin 은 속도 모드에서 마지막 명령을 유지하므로 매번 보낼 필요가
        없다. 매 콜백(50Hz)마다 전송하면 USB CAN 송신큐가 넘쳐 ENOBUFS 가 난다.
        """
        prev = self._sent.get(node_id)
        now = time.monotonic()
        keepalive_due = now - self._sent_time.get(node_id, 0.0) >= self.keepalive_interval
        force_zero = velocity == 0.0 and prev not in (None, 0.0)
        if (prev is None or force_zero or keepalive_due
                or abs(velocity - prev) >= self.send_threshold):
            if self.set_velocity(node_id, velocity):
                self._sent[node_id] = velocity
                self._sent_time[node_id] = now

    @staticmethod
    def _approach(current, target, max_step):
        diff = target - current
        if abs(diff) <= max_step:
            return target
        return current + (max_step if diff > 0.0 else -max_step)

    def _control_tick(self):
        if not self.initialized:
            return
        now = time.monotonic()
        if now - self.last_cmd_time > self.cmd_vel_timeout:
            self.last_linear = 0.0
            self.last_angular = 0.0
            for node_id in self.drive_motor_ids:
                self.last_vels[node_id] = 0.0
        # 롤러도 두절되면 멈춘다. cargo_load 가 적재 도중 죽으면 롤러만 계속
        # 도는 상태가 되기 때문이다. cargo_load 는 20Hz 로 재발행하므로
        # 정상 동작 중에는 걸리지 않는다.
        if self.roller_torque_enabled and now - self.last_roller_time > self.roller_timeout:
            self.get_logger().warn(
                f'roller_cmd {self.roller_timeout:.2f}s 두절 — 롤러 토크 해제')
            self._release_roller()
        dt = 1.0 / self.control_rate
        for node_id in self.motor_ids:
            current = self.applied_vels[node_id]
            target = self.last_vels[node_id]

            if node_id == self.roller_id:
                self.applied_vels[node_id] = target
                if self.roller_torque_enabled:
                    can_velocity = -target if self.roller_invert else target
                    self.send_velocity_if_changed(node_id, can_velocity)
                continue

            # 역회전은 먼저 정지한 뒤 반대 방향으로 가속한다.
            reversing = current * target < 0.0
            effective_target = 0.0 if reversing else target
            accelerating = abs(effective_target) > abs(current)
            limit = self.max_accel if accelerating else self.max_decel
            applied = self._approach(current, effective_target, limit * dt)
            self.applied_vels[node_id] = applied

            if node_id == self.roller_id:
                can_velocity = -applied if self.roller_invert else applied
            elif self.test_id > 0:
                can_velocity = self.sign * applied
            elif node_id == self.left_id:
                can_velocity = self.sign * applied
            else:
                can_velocity = applied
            self.send_velocity_if_changed(node_id, can_velocity)

    def _open_bus(self, initial=False):
        """CAN 버스를 연다. 실패하면 재시도 시각만 잡고 False 를 돌려준다."""
        try:
            self.bus = can.interface.Bus(
                channel=self.can_channel, interface="socketcan")
        except Exception as exc:
            self.bus = None
            message = f'CAN unavailable on {self.can_channel}: {exc}'
            if initial and self.require_can:
                self.get_logger().fatal(message)
                raise RuntimeError(message) from exc
            self._can_reconnect_at = (
                time.monotonic() + self.can_reconnect_interval)
            if initial:
                self.get_logger().warn(message + ' (safe disconnected mode)')
            elif not self._can_lost_logged:
                self.get_logger().warn(message + ' — 재연결 대기')
                self._can_lost_logged = True
            return False
        self.get_logger().info(f'CAN bus initialized on {self.can_channel}')
        self._can_backoff_until = 0.0
        self._can_lost_logged = False
        return True

    def _drop_bus(self, reason):
        """죽은 소켓을 놓고 재연결을 예약한다. 노드는 살려 둔다."""
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.bus = None
        self.initialized = False
        for status in self.axis_status.values():
            status['current_a'] = None
            status['current_time'] = None
        self._can_reconnect_at = time.monotonic() + self.can_reconnect_interval
        self.get_logger().error(
            f'CAN 연결 끊김 ({reason}) — {self.can_reconnect_interval:.1f}초 뒤 재연결 시도')

    def _read_can_status(self):
        """CAN heartbeat를 소진하고 axis error/state를 기록한다."""
        if self.bus is None:
            # 어댑터가 돌아왔는지 주기적으로 확인한다.
            if time.monotonic() >= self._can_reconnect_at:
                if self._open_bus():
                    self.initialized = self.initialize_motors()
                    self.get_logger().info(
                        f'CAN 재연결 완료 — 모터 재초기화 {self.initialized}')
                else:
                    self._can_reconnect_at = (
                        time.monotonic() + self.can_reconnect_interval)
            return
        for _ in range(20):
            try:
                msg = self.bus.recv(timeout=0.0)
            except Exception as exc:
                self._drop_bus(f'recv: {exc}')
                return
            if msg is None:
                break
            node_id = msg.arbitration_id >> 5
            cmd_id = msg.arbitration_id & 0x1F
            if cmd_id == 0x01 and node_id in self.axis_status and len(msg.data) >= 5:
                self.last_can_response_time = time.monotonic()
                self.axis_status[node_id]['axis_error'] = struct.unpack('<I', bytes(msg.data[:4]))[0]
                self.axis_status[node_id]['axis_state'] = int(msg.data[4])
            elif cmd_id == 0x09 and node_id in self.axis_status and len(msg.data) >= 8:
                now = time.monotonic()
                self.last_can_response_time = now
                position, velocity = struct.unpack('<ff', bytes(msg.data[:8]))
                self.axis_status[node_id]['position'] = float(position)
                self.axis_status[node_id]['encoder_velocity'] = float(velocity)
                self.axis_status[node_id]['encoder_time'] = now
            elif cmd_id == 0x14 and node_id in self.axis_status and len(msg.data) >= 8:
                now = time.monotonic()
                self.last_can_response_time = now
                iq_setpoint, iq_measured = struct.unpack('<ff', bytes(msg.data[:8]))
                self.axis_status[node_id]['iq_setpoint'] = float(iq_setpoint)
                self.axis_status[node_id]['current_a'] = float(iq_measured)
                self.axis_status[node_id]['current_time'] = now
                self.current_seq += 1

    def _clamp(self, v):
        return max(-self.max_vel, min(self.max_vel, v))

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.monotonic()
        self.last_linear = msg.linear.x
        self.last_angular = msg.angular.z
        cmd = (round(self.last_linear, 3), round(self.last_angular, 3))
        if cmd != self._logged_cmd:
            self.get_logger().info(
                f'[CMD/PI] from HUB /cmd_vel: lin={self.last_linear:+.2f} '
                f'ang={self.last_angular:+.2f}')
            self._logged_cmd = cmd

        if self.test_id > 0:
            vel = self._clamp(self.last_linear * self.vel_scale)
            self.last_vels[self.test_id] = vel
        else:
            left = self._clamp((self.last_linear - self.last_angular) * self.vel_scale)
            right = self._clamp((self.last_linear + self.last_angular) * self.vel_scale)
            self.last_vels[self.left_id] = left
            self.last_vels[self.right_id] = right

    def _release_roller(self):
        """롤러를 세우고 토크를 뗀다. 정지 명령과 두절 정지가 같이 쓴다."""
        self.last_vels[self.roller_id] = 0.0
        self.applied_vels[self.roller_id] = 0.0
        if not self.roller_torque_enabled:
            return
        self.set_velocity(self.roller_id, 0.0)
        self.set_idle(self.roller_id)
        self.roller_torque_enabled = False
        self._sent.pop(self.roller_id, None)
        self._sent_time.pop(self.roller_id, None)
        self.get_logger().info(
            f'Roller ID={self.roller_id}: torque released (Axis Idle)')

    def roller_cmd_callback(self, msg):
        """독립 롤러 명령. 0이면 토크 해제, 그 외에는 즉시 속도 적용."""
        self.last_roller_time = time.monotonic()
        if not self.initialized:
            return
        normalized = max(-1.0, min(1.0, float(msg.data)))
        target = normalized * self.roller_scale
        target = max(-self.roller_max_vel, min(self.roller_max_vel, target))

        if abs(target) < 1e-6:
            self._release_roller()
            return

        if not self.roller_torque_enabled:
            self.clear_errors(self.roller_id)
            self.set_controller_velocity(self.roller_id)
            self.set_closed_loop(self.roller_id)
            self.roller_torque_enabled = True
            self._sent.pop(self.roller_id, None)
            self._sent_time.pop(self.roller_id, None)
            self.get_logger().info(
                f'Roller ID={self.roller_id}: torque enabled (Closed Loop)')
        self.last_vels[self.roller_id] = target

    def _log_status(self):
        vels = ', '.join(
            f'ID{i} target={self.last_vels[i]:+.2f} applied={self.applied_vels[i]:+.2f}rev/s'
            for i in self.motor_ids)
        self.get_logger().info(
            f'[MOTOR/PI] recv /cmd_vel: lin={self.last_linear:+.2f} '
            f'ang={self.last_angular:+.2f} | -> motor {vels}')
        self._publish_status()

    def _forward_revs(self, node_id):
        position = self.axis_status[node_id].get('position')
        if position is None:
            return None
        if self.test_id > 0 or node_id == self.left_id:
            return self.sign * position
        return position

    def _publish_status(self):
        now = time.monotonic()
        can_age = (None if self.last_can_response_time is None else
                   max(0.0, now - self.last_can_response_time))
        can_interface_open = self.bus is not None
        can_responsive = (
            self.bus is not None and can_age is not None and
            can_age <= self.can_response_timeout)

        def motor_status(node_id):
            values = dict(self.axis_status[node_id])
            current_updated = values.pop('current_time')
            current_age = (None if current_updated is None else
                           max(0.0, now - current_updated))
            current_fresh = (
                current_age is not None and
                current_age <= self.can_response_timeout)
            encoder_updated = values.pop('encoder_time')
            encoder_age = (None if encoder_updated is None else
                           max(0.0, now - encoder_updated))
            encoder_fresh = (
                encoder_age is not None and
                encoder_age <= self.can_response_timeout)
            if not current_fresh:
                values['current_a'] = None
                values['iq_setpoint'] = None
            if not encoder_fresh:
                values['position'] = None
                values['encoder_velocity'] = None
            return {
                'target': self.last_vels[node_id],
                'applied': self.applied_vels[node_id],
                'forward_revs': (
                    self._forward_revs(node_id) if encoder_fresh else None),
                **values,
                'current_fresh': current_fresh,
                'current_age_s': current_age,
                'encoder_fresh': encoder_fresh,
                'encoder_age_s': encoder_age,
            }

        fresh_drive_current_ids = [
            node_id for node_id in self.drive_motor_ids
            if self.axis_status[node_id]['current_time'] is not None
            and now - self.axis_status[node_id]['current_time']
            <= self.can_response_timeout
        ]

        status = String()
        status.data = json.dumps({
            'source': 'pi',
            # 화면의 CAN 연결 표시는 SocketCAN 인터페이스/소켓 상태다.
            # 모터 컨트롤러의 실제 응답 상태는 can_responsive와 전류 freshness로
            # 별도 제공해, 어댑터 연결과 장치 응답을 혼동하지 않는다.
            'can_interface_open': can_interface_open,
            'can_connected': can_interface_open,
            'can_responsive': can_responsive,
            'can_response_age_s': can_age,
            'current_monitor_enabled': self.enable_rtr_requests,
            'current_request_rate_hz': self.current_request_rate,
            'current_monitor_ready': (
                len(fresh_drive_current_ids) == len(self.drive_motor_ids)),
            'current_fresh_drive_ids': fresh_drive_current_ids,
            'current_seq': self.current_seq,
            'contact_current_threshold_a': self.contact_current_threshold_a,
            'left_id': self.left_id,
            'right_id': self.right_id,
            'drive_motor_ids': list(self.drive_motor_ids),
            'cmd_vel': {'linear': self.last_linear, 'angular': self.last_angular},
            'roller_torque_enabled': self.roller_torque_enabled,
            'motors': {
                str(i): motor_status(i) for i in self.motor_ids
            },
        })
        self.status_pub.publish(status)

    def shutdown(self):
        self.get_logger().info('Shutting down...')
        if self.bus is None:
            return
        for node_id in self.motor_ids:
            self.set_velocity(node_id, 0.0)
        self.set_idle(self.roller_id)
        self.roller_torque_enabled = False
        self.bus.shutdown()
        self.bus = None


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    def signal_handler(sig, frame):
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
