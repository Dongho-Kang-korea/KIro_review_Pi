"""1호기의 RC(무선 조종기) 수신기 채널 값을 받아 이 호기를 조종한다.

RC 수신기는 **1호기에만** 물려 있다. 1호기가 `/receiver/channels`
(``std_msgs/UInt16MultiArray``, 14채널, 각 1000~2000·중립 1500)를 그대로
발행하므로, 이 노드는 그걸 직접 구독해 CH7(호기 선택)로 "지금 이 채널이
2호기 몫인지"를 스스로 걸러낸다. 별도의 호기별 중계 토픽은 없다 — 2026-08-30
1호기 쪽 코드(`manual.py`, `/receiver/channels`·`/receiver/status`·
`/cmd_vel_rc` 등)를 그대로 확인하고 맞췄다(``TODO.md`` 14번 경위 참고).

채널 배치(2026-08-30 1호기·2호기 합의):

===  =====================  ================================================
CH   기능                    2호기 처리
===  =====================  ================================================
1    주행 좌우(angular)       steering, 데드밴드 정규화
2    주행 전후(linear)        throttle, 데드밴드 정규화
3    (1호기 카메라 틸트)      2호기는 **솔레노이드**로 재사용 — 중앙값 임계,
                            스틱 내림=잠금(HIGH)/올림=해제(LOW)
4    미사용                  무시
5    VRA(속도 스케일 노브)    steering/throttle에 곱하는 0.0~1.0 배율
6    VRB(폐기)               무시(1호기도 폐기 — CH9가 고정속 스위치가 되며 무의미해짐)
7    호기 선택 스위치         ≥1650 이면 "2호기" — 그 미만(중간 구간 포함)은
                            전부 "2호기 아님"으로 안전하게 처리
8    정지/재생 게이트         ≥1650 이어야 "재생" — 그 미만은 정지로 취급
9    컨베이어 3단 스위치      <1250 CCW(-1.0) / >1750 CW(+1.0) / 그 사이 정지
10   자동/수동                ≥1500 이면 2호기를 ``auto``로, 아니면 ``rc``로
                            원격 전환(drive/mode_cmd)
===  =====================  ================================================

CH7 이탈 시 동작(2026-08-30 확정): **마지막 값을 유지**한다 — CH7이 1호기
쪽으로 가 있어도 2호기는 직전에 받은 조향/스로틀/컨베이어/모드를 계속
낸다. 0으로 완전히 끊기는 건 오직 **`/receiver/channels` 토픽 자체가
끊겼을 때**뿐이다(수신기 연결 끊김) — 이때는 이 노드가 새 프레임을 못
받으니 재발행도 멈추고, cmd_vel/rc는 arbiter의 ``source_timeout_s``가,
컨베이어는 ``cargo_load``의 수동 롤러 timeout이 각자 알아서 0으로 내린다
(이 노드가 직접 손 안 대도 됨 — 기존 안전장치를 그대로 재사용). 예외는
솔레노이드: 자체 timeout이 없는 단순 래치라(``base/solenoid.py``) 신호가
끊기면 이 노드가 직접 안전 기본값(잠금)으로 되돌린다.

웹 콘솔의 "조종 방식"(웹/RC) 토글은 이 노드와 별개 채널이다
(``rc/enabled``, Bool) — 꺼져 있으면 채널 데이터가 와도 전부 무시하고,
켜졌다가 꺼지는 순간에는 "마지막 값 유지"가 아니라 **즉시 0**으로
끊는다(사람이 명시적으로 통제권을 가져간 것이므로).

.. warning::
   CH7/CH8/CH9/CH10 임계값과 CH3(솔레노이드)/CH1/CH2 방향은 1호기 쪽
   합의안을 반영한 것이지 실물로 확인된 값은 아니다. 배선 후 로그로
   raw 채널 값이 기대한 방향으로 오는지 먼저 확인할 것 — ``TODO.md``
   14번 참고.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String, UInt16MultiArray

from fleet import protocol


class RcBridge(Node):
    def __init__(self):
        super().__init__('fleet_rc_bridge')

        self.declare_parameter('unit', 2)
        # 비어 있으면 protocol.TOPIC_RECEIVER_CHANNELS를 쓴다. 1호기 쪽
        # 토픽 이름이 바뀌면 이 파라미터만 바꾸면 되고 코드 수정은 필요 없다.
        self.declare_parameter('topic', '')
        self.declare_parameter('channel_count', 14)
        self.declare_parameter('pwm_min', 1000.0)
        self.declare_parameter('pwm_max', 2000.0)
        self.declare_parameter('deadband_us', 20.0)

        self.declare_parameter('steering_channel', 1)
        self.declare_parameter('throttle_channel', 2)
        self.declare_parameter('steering_reversed', False)
        self.declare_parameter('throttle_reversed', False)

        self.declare_parameter('solenoid_channel', 3)
        self.declare_parameter('solenoid_reversed', False)

        self.declare_parameter('speed_scale_channel', 5)

        self.declare_parameter('robot_select_channel', 7)
        self.declare_parameter('robot_select_threshold', 1650.0)

        self.declare_parameter('stop_channel', 8)
        self.declare_parameter('run_threshold', 1650.0)

        self.declare_parameter('conveyor_channel', 9)
        self.declare_parameter('conveyor_low_threshold', 1250.0)
        self.declare_parameter('conveyor_high_threshold', 1750.0)

        self.declare_parameter('mode_channel', 10)
        self.declare_parameter('mode_auto_threshold', 1500.0)

        # manual(키보드) 상한과 동일하게 시작한다 — VRA 스케일(0~1)이
        # 여기 곱해지므로 이게 "최고 속도"다.
        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 0.50)

        # 이 시간 안에 새 채널 프레임이 안 오면 "수신기 연결 끊김"으로
        # 본다 — 솔레노이드 안전 복귀 판정에만 쓴다(주행/컨베이어는 각자
        # 알아서 timeout이 있어 이 값과 무관).
        self.declare_parameter('signal_timeout_s', 1.0)
        self.declare_parameter('solenoid_failsafe_lock_on_timeout', True)

        self.unit = int(self.get_parameter('unit').value)
        topic = str(self.get_parameter('topic').value).strip()
        self.topic = topic or protocol.TOPIC_RECEIVER_CHANNELS
        self.channel_count = int(self.get_parameter('channel_count').value)
        self.pwm_min = float(self.get_parameter('pwm_min').value)
        self.pwm_max = float(self.get_parameter('pwm_max').value)
        self.deadband_us = float(self.get_parameter('deadband_us').value)

        self.steering_channel = int(self.get_parameter('steering_channel').value)
        self.throttle_channel = int(self.get_parameter('throttle_channel').value)
        self.steering_reversed = bool(self.get_parameter('steering_reversed').value)
        self.throttle_reversed = bool(self.get_parameter('throttle_reversed').value)

        self.solenoid_channel = int(self.get_parameter('solenoid_channel').value)
        self.solenoid_reversed = bool(self.get_parameter('solenoid_reversed').value)

        self.speed_scale_channel = int(self.get_parameter('speed_scale_channel').value)

        self.robot_select_channel = int(self.get_parameter('robot_select_channel').value)
        self.robot_select_threshold = float(
            self.get_parameter('robot_select_threshold').value)

        self.stop_channel = int(self.get_parameter('stop_channel').value)
        self.run_threshold = float(self.get_parameter('run_threshold').value)

        self.conveyor_channel = int(self.get_parameter('conveyor_channel').value)
        self.conveyor_low = float(self.get_parameter('conveyor_low_threshold').value)
        self.conveyor_high = float(self.get_parameter('conveyor_high_threshold').value)

        self.mode_channel = int(self.get_parameter('mode_channel').value)
        self.mode_auto_threshold = float(
            self.get_parameter('mode_auto_threshold').value)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.signal_timeout_s = float(self.get_parameter('signal_timeout_s').value)
        self.solenoid_failsafe = bool(
            self.get_parameter('solenoid_failsafe_lock_on_timeout').value)

        self._needed_channels = max(
            self.steering_channel, self.throttle_channel, self.solenoid_channel,
            self.speed_scale_channel, self.robot_select_channel, self.stop_channel,
            self.conveyor_channel, self.mode_channel)

        # "유지" 대상 상태 — CH7이 2호기를 안 가리켜도 이 값들을 계속
        # 재발행해서 arbiter/cargo_load의 자체 timeout에 안 걸리게 한다.
        self.last_twist = Twist()
        self.last_roller = 0.0
        self.last_mode_sent = None
        self.last_solenoid_locked = None
        self.robot_selected = False
        self.run_active = False
        self.last_channel_time = None
        self.enabled = False   # 웹 토글 기본값 — 명시적으로 켜야 반응한다.

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/rc', 10)
        self.roller_pub = self.create_publisher(Float32, 'roller_manual_cmd', 10)
        self.solenoid_pub = self.create_publisher(Bool, 'solenoid_cmd', 10)
        self.mode_pub = self.create_publisher(String, 'drive/mode_cmd', 10)
        self.status_pub = self.create_publisher(String, 'rc/status', 10)

        self.create_subscription(UInt16MultiArray, self.topic, self._on_channels, 10)
        self.create_subscription(Bool, 'rc/enabled', self._on_enabled, 10)
        self.create_timer(0.2, self._watchdog)
        self.create_timer(0.2, self._publish_status)

        self.get_logger().warn(
            f'rc_bridge: topic={self.topic!r} — CH1/2/3/5/7/8/9/10 매핑과 임계값은'
            ' 1호기 합의안 기반 미검증 값. 배선 후 raw 채널 로그로 방향부터'
            ' 확인할 것 (TODO.md 14번).')

    # ---------- 변환 ----------

    def _normalize(self, raw, reversed_):
        """raw(pwm_min~pwm_max) 를 데드밴드를 뺀 -1.0~1.0 로 바꾼다."""
        center = (self.pwm_min + self.pwm_max) / 2.0
        half_range = max(1e-6, (self.pwm_max - self.pwm_min) / 2.0)
        offset = float(raw) - center
        if abs(offset) <= self.deadband_us:
            value = 0.0
        else:
            sign = 1.0 if offset > 0 else -1.0
            span = max(1e-6, half_range - self.deadband_us)
            value = sign * (abs(offset) - self.deadband_us) / span
        value = max(-1.0, min(1.0, value))
        return -value if reversed_ else value

    def _knob_scale(self, raw):
        """1호기 manual.py의 knob_scale()과 동일 — 선형, 데드존 없음."""
        scale = (float(raw) - self.pwm_min) / max(1e-6, (self.pwm_max - self.pwm_min))
        return max(0.0, min(1.0, scale))

    def _conveyor_value(self, raw):
        if raw < self.conveyor_low:
            return -1.0
        if raw > self.conveyor_high:
            return 1.0
        return 0.0

    def _solenoid_locked(self, raw):
        """중앙값 임계 — 스틱을 내리면(raw < 중앙) 잠금, 올리면 해제."""
        center = (self.pwm_min + self.pwm_max) / 2.0
        want_lock = raw < center
        return (not want_lock) if self.solenoid_reversed else want_lock

    # ---------- 수신 ----------

    def _on_enabled(self, msg):
        was_enabled = self.enabled
        self.enabled = bool(msg.data)
        if was_enabled and not self.enabled:
            # 사람이 명시적으로 웹으로 통제권을 가져간 순간이다 — CH7
            # 이탈과 달리 "유지"하지 않고 즉시 0으로 끊는다. 솔레노이드는
            # 예외(코드 위 docstring 참고) — 건드리지 않는다.
            self.last_twist = Twist()
            self.last_roller = 0.0
            self.last_mode_sent = None
            self.robot_selected = False
            self.run_active = False
            self.cmd_pub.publish(Twist())
            self.roller_pub.publish(Float32(data=0.0))
            self.get_logger().info('rc_bridge: 웹 토글 OFF — 조종기 입력 즉시 무시, 주행/컨베이어 0')
        elif self.enabled and not was_enabled:
            self.get_logger().info('rc_bridge: 웹 토글 ON — 조종기 입력 반영 시작')

    def _on_channels(self, msg):
        now = time.monotonic()
        self.last_channel_time = now
        channels = list(msg.data)

        if len(channels) < self._needed_channels:
            self.get_logger().warn(
                f'rc_bridge: 채널 수 부족(받은 {len(channels)}개, '
                f'{self._needed_channels}개 필요) — 이번 프레임 버림')
            return
        if len(channels) != self.channel_count:
            self.get_logger().warn(
                f'rc_bridge: 채널 수가 channel_count({self.channel_count})와'
                f' 다르다(받은 {len(channels)}개) — 배선/설정을 확인할 것',
                throttle_duration_sec=5.0)

        if not self.enabled:
            return  # 웹 토글이 꺼져 있으면 채널 자체를 무시한다.

        self.robot_selected = (
            channels[self.robot_select_channel - 1] >= self.robot_select_threshold)
        self.run_active = channels[self.stop_channel - 1] >= self.run_threshold

        if self.robot_selected:
            if self.run_active:
                scale = self._knob_scale(channels[self.speed_scale_channel - 1])
                steering = self._normalize(
                    channels[self.steering_channel - 1], self.steering_reversed)
                throttle = self._normalize(
                    channels[self.throttle_channel - 1], self.throttle_reversed)
                twist = Twist()
                twist.linear.x = throttle * self.max_linear * scale
                twist.angular.z = steering * self.max_angular * scale
                self.last_twist = twist
                self.last_roller = self._conveyor_value(
                    channels[self.conveyor_channel - 1])
                want_lock = self._solenoid_locked(
                    channels[self.solenoid_channel - 1])
                if want_lock != self.last_solenoid_locked:
                    self.solenoid_pub.publish(Bool(data=want_lock))
                    self.last_solenoid_locked = want_lock
            else:
                # CH8='정지' — 이동만 0. 솔레노이드(커플링 잠금 상태)는
                # 건드리지 않는다(정지는 주행 개념이지 결합 개념이 아니다).
                self.last_twist = Twist()
                self.last_roller = 0.0

            desired_mode = ('auto' if channels[self.mode_channel - 1] >=
                             self.mode_auto_threshold else 'rc')
            if desired_mode != self.last_mode_sent:
                self.mode_pub.publish(String(data=desired_mode))
                self.last_mode_sent = desired_mode
        # else: CH7이 2호기를 안 가리킨다 — last_twist/last_roller/모드/
        # 솔레노이드 전부 지금 값 그대로 "유지"(아무것도 안 바꿈).

        # CH7 선택 여부와 무관하게 매 프레임 재발행 — 이게 "유지"의 실체다.
        # 진짜 끊김(수신기 연결 끊김)은 이 콜백 자체가 안 불려서 자동으로
        # 재발행이 멈추고, 그다음은 arbiter/cargo_load의 자체 timeout이
        # 맡는다.
        self.cmd_pub.publish(self.last_twist)
        self.roller_pub.publish(Float32(data=self.last_roller))

    # ---------- 안전 ----------

    def _watchdog(self):
        if not self.enabled or self.last_channel_time is None:
            return
        age = time.monotonic() - self.last_channel_time
        if age <= self.signal_timeout_s:
            return
        # 여기서부터는 cmd_vel/rc·roller_manual_cmd를 이 노드가 더 이상
        # 재발행하지 않으므로(콜백이 안 불림) arbiter/cargo_load의 기존
        # timeout이 알아서 0으로 내린다 — 별도 처리 불필요. 솔레노이드만
        # 자체 timeout이 없는 래치라 여기서 직접 안전 상태로 되돌린다.
        if self.solenoid_failsafe and self.last_solenoid_locked is not True:
            self.solenoid_pub.publish(Bool(data=True))
            self.last_solenoid_locked = True
            self.get_logger().warn(
                'rc_bridge: RC 신호 두절(%.1fs) — 안전을 위해 솔레노이드 강제 잠금'
                % age)

    def _publish_status(self):
        age = (time.monotonic() - self.last_channel_time
               if self.last_channel_time is not None else None)
        connected = age is not None and age <= self.signal_timeout_s
        payload = {
            't': time.time(), 'enabled': self.enabled, 'connected': connected,
            'age_s': None if age is None else round(age, 2),
            'robot_selected': self.robot_selected, 'run_active': self.run_active,
            'requested_mode': self.last_mode_sent,
            'solenoid_locked': self.last_solenoid_locked,
            'linear': self.last_twist.linear.x, 'angular': self.last_twist.angular.z,
            'roller': self.last_roller,
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))


def main(args=None):
    rclpy.init(args=args); node = RcBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
