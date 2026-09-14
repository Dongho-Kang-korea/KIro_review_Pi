"""2호기 통신 명령을 수신하고 시간 초과 시 정지한다."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from fleet import protocol


class Follower(Node):
    def __init__(self):
        super().__init__('fleet_follower')

        # 인수인계 수정 경계:
        # - status/command/state timeout은 fleet/config/follower.yaml에서 조정한다.
        # - /fleet/* 토픽과 JSON 규약은 1호기 fleet/protocol.py와의 계약이므로
        #   한 호기만 바꾸지 않는다. 명령 갱신이 끊기면 정지하는 원칙도 유지한다.
        self.declare_parameter('unit', 2)
        self.declare_parameter('status_rate_hz', protocol.STATUS_RATE_HZ)
        self.declare_parameter('command_timeout_s', protocol.CMD_TIMEOUT_S)
        self.declare_parameter('state_timeout_s', 2.5)
        self.unit = int(self.get_parameter('unit').value)
        if self.unit != 2:
            # 3호기는 설계에서 빠졌다 (TODO 19, 2026-08-31 사용자 확정).
            # 여름 적재는 1·2호기 둘이서 한다.
            raise ValueError('fleet follower unit must be 2')
        self.command_timeout = float(self.get_parameter('command_timeout_s').value)
        self.state_timeout = float(self.get_parameter('state_timeout_s').value)
        self.last_cmd_monotonic = None
        self.last_state_monotonic = None
        self.last_seq = -1
        self.action = 'idle'
        self.phase = 'idle'
        self.state = {}
        self.cargo_target = {}
        self.motor = {}
        self.marker = {}
        self.uwb_mm = None
        self.heading_deg = None
        self.roller = 0.0
        self.solenoid_locked = False
        self.red_detected = False
        self.watchdog_stopped = False

        self.action_pub = self.create_publisher(String, 'fleet/action', 10)
        self.stop_pub = self.create_publisher(Bool, 'safety/stop', 10)
        self.status_pub = self.create_publisher(
            String, protocol.TOPIC_STATUS.format(self.unit), 10)
        self.create_subscription(
            String, protocol.TOPIC_CMD.format(self.unit), self._on_command, 10)
        self.create_subscription(String, protocol.TOPIC_STATE, self._on_state, 10)
        self.create_subscription(
            String, protocol.TOPIC_CARGO_TARGET, self._on_cargo_target, 10)
        self.create_subscription(String, 'mission/phase', self._on_phase, 10)
        self.create_subscription(String, 'motor/status', self._on_motor, 10)
        self.create_subscription(String, 'marker/status', self._on_marker, 10)
        self.create_subscription(Float32, 'uwb/distance_mm', self._on_uwb, 10)
        self.create_subscription(Float32, 'imu/heading_deg', self._on_heading, 10)
        self.create_subscription(Float32, 'roller_cmd', self._on_roller, 10)
        self.create_subscription(Bool, 'solenoid_status', self._on_solenoid, 10)
        self.create_subscription(Bool, 'cargo/red_detected', self._on_red, 10)
        rate = max(1.0, float(self.get_parameter('status_rate_hz').value))
        self.create_timer(0.05, self._watchdog)
        self.create_timer(1.0 / rate, self._publish_status)

    def _publish_action(self, values):
        msg = String()
        msg.data = json.dumps(values, separators=(',', ':'))
        self.action_pub.publish(msg)

    def _on_command(self, msg):
        now = time.monotonic()
        try:
            value = protocol.validate_command(protocol.decode(msg.data))
        except protocol.ProtocolError as exc:
            self.get_logger().warn(f'Rejected fleet command: {exc}')
            return
        self.last_cmd_monotonic = now
        if value['action'] == 'stop':
            self.action = 'stop'
            self.last_seq = max(self.last_seq, value['seq'])
            self.watchdog_stopped = True
            self._publish_action(value)
            self._publish_stop(True)
            return
        if value['seq'] <= self.last_seq:
            return
        self.last_seq = value['seq']
        self.action = value['action']
        self.watchdog_stopped = False
        self._publish_stop(False)
        self._publish_action(value)

    def _on_state(self, msg):
        try:
            value = protocol.validate_state(protocol.decode(msg.data))
        except protocol.ProtocolError as exc:
            self.get_logger().warn(f'Rejected fleet state: {exc}')
            return
        self.state = value
        self.last_state_monotonic = time.monotonic()
        # 여름 적재만 개별 명령을 요구하고 나머지는 공용 추종을 사용한다.
        if value['section'] != 'summer' and self.action in ('idle', 'follow', 'stop'):
            desired = 'follow' if value.get('phase') != 'finished' else 'idle'
            if desired != self.action:
                self.action = desired
                self._publish_action({
                    't': value['t'], 'seq': self.last_seq,
                    'action': desired, 'source': 'fleet_state'})

    def _on_cargo_target(self, msg):
        try:
            self.cargo_target = protocol.validate_cargo_target(
                protocol.decode(msg.data))
        except protocol.ProtocolError as exc:
            self.get_logger().warn(f'Rejected cargo target: {exc}')

    def _watchdog(self):
        now = time.monotonic()
        command_required = self.state.get('section') == 'summer' or self.action not in ('idle', 'follow')
        cmd_stale = command_required and (
            self.last_cmd_monotonic is None or
            now - self.last_cmd_monotonic > self.command_timeout)
        state_stale = self.action == 'follow' and (
            self.last_state_monotonic is None or
            now - self.last_state_monotonic > self.state_timeout)
        if (cmd_stale or state_stale) and not self.watchdog_stopped:
            self.watchdog_stopped = True
            self.action = 'stop'
            self._publish_action({
                't': time.time(), 'seq': self.last_seq,
                'action': 'stop', 'reason': 'fleet_timeout'})
            self._publish_stop(True)

    def _publish_stop(self, stopped):
        msg = Bool(); msg.data = bool(stopped); self.stop_pub.publish(msg)

    def _on_phase(self, msg):
        value = str(msg.data).strip().lower()
        if value in protocol.PHASES:
            self.phase = value

    def _on_motor(self, msg):
        try: self.motor = json.loads(msg.data)
        except Exception: pass

    def _on_marker(self, msg):
        try: self.marker = json.loads(msg.data)
        except Exception: pass

    def _on_uwb(self, msg): self.uwb_mm = float(msg.data)
    def _on_heading(self, msg): self.heading_deg = float(msg.data)
    def _on_roller(self, msg): self.roller = float(msg.data)
    def _on_solenoid(self, msg): self.solenoid_locked = bool(msg.data)
    def _on_red(self, msg): self.red_detected = bool(msg.data)

    def _current(self):
        # 적재 완료 판정에 쓰이므로 롤러 전류를 섞지 않는다.
        drive_ids = {
            str(value) for value in self.motor.get('drive_motor_ids', [])}
        if not drive_ids:
            return None
        values = []
        for motor_id, item in self.motor.get('motors', {}).items():
            value = item.get('current_a')
            if (str(motor_id) in drive_ids and
                    item.get('current_fresh', True) and
                    isinstance(value, (int, float)) and math.isfinite(value)):
                values.append(abs(float(value)))
        return max(values) if values else None

    def _publish_status(self):
        drive_current = self._current()
        payload = {
            't': time.time(), 'unit': self.unit, 'phase': self.phase,
            'seq_ack': self.last_seq, 'uwb_mm': self.uwb_mm,
            # current_a는 구버전 호환용이고, 새 코드는 명시적인 필드를 쓴다.
            'current_a': drive_current,
            'drive_current_a': drive_current,
            'roller': self.roller,
            'red_detected': self.red_detected,
            'marker_detected': bool(self.marker.get('detected', False)),
            'heading_deg': self.heading_deg,
            'solenoid_locked': self.solenoid_locked,
            'action': self.action, 'watchdog_stopped': self.watchdog_stopped,
        }
        msg = String(); msg.data = json.dumps(payload, separators=(',', ':'))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = Follower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node._publish_stop(True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
