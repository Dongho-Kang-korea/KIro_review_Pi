"""1호기가 지휘하는 2호기 여름 화물 적재 상태기계.

``approach``를 받으면 2호기는 물자 쪽으로 이동하지 않고 즉시 180도
회전한다. 회전 뒤에는 정지한 채 1호기가 물자를 밀어 오기를 기다린다.
UWB가 접촉 감시 거리 안이고 1호기가 자기 전류 기준으로 접촉을 보고하면
``contact``가 된다. 이후 ``load`` 명령을 받으면 2호기도 저속 후진하며
롤러를 돌린다. 1호기는 자기 전류 하락 판정 결과를 통신으로 보내고,
2호기는 자기 주행 전류가 설정한 상승 기준을 먼저 넘은 뒤 하락 기준
아래에서 설정 시간 유지되는지 본다. 둘 중 하나가 확인되면 적재를 멈추고
``loaded``를 보고한다.

``couple``은 별도 ``coupling.py`` 상태기계에 시작 명령만 전달한다.
"""
import json
import math
import time

import cv2
# 영상 노드가 여러 개 동시에 돌아 4코어에 스레드가 몰린다. 프로세스
# 단위로 이미 병렬이므로 OpenCV 내부 스레드는 1로 묶는 편이 총
# 처리량이 낫다(2026-09-01 부하 실측).
cv2.setNumThreads(1)
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String


def angle_delta(current, origin):
    return abs((current - origin + 180.0) % 360.0 - 180.0)


class CargoLoad(Node):
    def __init__(self):
        super().__init__('cargo_load')

        # 모든 수치는 mission/config/cargo.yaml에서 실차에 맞춰 조정한다.
        parameters = {
            'turn_speed_rps': 0.28, 'turn_target_deg': 180.0,
            'turn_tolerance_deg': 8.0,
            'load_speed_mps': -0.035, 'release_roller': -0.6,
            'sensor_timeout_s': 0.5,
            # 이 거리 안에서만 1호기가 보낸 접촉 판정을 인정한다.
            'contact_uwb_threshold_mm': 900.0,
            'contact_confirm_samples': 3,
            # 2호기 전류 기준만 이 설정이 소유한다. 1호기는 자기 설정으로
            # 판정한 결과를 /fleet/status/unit1에 싣는다.
            'unit2_load_high_current_a': 0.0,
            'unit2_load_low_current_a': 0.0,
            'load_min_duration_s': 1.0,
            'load_complete_hold_s': 2.0,
            'loading_timeout_s': 20.0,
            'red_min_area': 1200.0,
            'red_h_min_1': 0, 'red_h_max_1': 12,
            'red_h_min_2': 165, 'red_h_max_2': 179,
            'red_s_min': 90, 'red_v_min': 70,
            # 웹 콘솔의 수동 컨베이어 조작 — 자동 미션이 idle/stop일 때만
            # 채택되고, 이 시간 안에 새 명령이 없으면 자동으로 0이 된다
            # (수동 주행 cmd_vel/manual과 같은 안전 패턴).
            'manual_roller_timeout_s': 0.4,
            # 적색 화물 검출을 초당 몇 번 돌릴지. 예전에는 카메라 프레임
            # 30fps 마다 1280x720 JPEG 디코딩 + HSV + 컨투어를 전부
            # 돌려서 코어 하나의 절반을 미션과 무관하게 항상 먹었다
            # (2026-09-01 실측: cargo_load 52%, 전체 load 17/4코어).
            # 화물은 빠르게 움직이지 않으므로 이 주기면 충분하고,
            # cargo/red_detected 계약은 그대로다.
            'red_detect_rate_hz': 5.0,
        }
        for name, value in parameters.items(): self.declare_parameter(name, value)
        self.action = 'idle'
        self.phase = 'idle'
        self.roller_value = 0.0
        self.last_red_detect = 0.0
        self.manual_roller = None
        self.manual_roller_time = None
        self.uwb_mm = None
        self.uwb_time = None
        self.heading = None
        self.heading_time = None
        self.turn_origin = None
        self.turn_target_deg = float(self.get_parameter('turn_target_deg').value)
        self.turn_next_phase = 'idle'
        self.current_a = None
        self.current_time = None
        self.current_update = 0
        self.last_motor_sequence = None
        self.drive_motor_ids = []
        self.leader_current_a = None
        self.leader_status_time = None
        self.leader_status_update = 0
        self.last_leader_status_update = -1
        self.leader_current_source = ''
        self.leader_contact_detected = False
        self.leader_load_current_dropped = False
        self.leader_current_detection_ready = False
        self.contact_hits = 0
        self.load_requested = False
        self.loading_started = None
        self.load_high_seen = False
        self.load_drop_since = None
        self.reason = ''
        self.red_detected = False
        self.safety_stop = False
        self.emergency_stop = False
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/cargo', 10)
        self.roller_pub = self.create_publisher(Float32, 'roller_cmd', 10)
        self.red_pub = self.create_publisher(Bool, 'cargo/red_detected', 10)
        self.status_pub = self.create_publisher(String, 'mission/cargo/status', 10)
        self.create_subscription(String, 'mission/action', self._on_action, 10)
        self.create_subscription(
            Float32, 'roller_manual_cmd', self._on_roller_manual, 10)
        self.create_subscription(
            String, '/fleet/status/unit1', self._on_leader_status, 10)
        self.create_subscription(Float32, 'uwb/distance_mm', self._on_uwb, 10)
        self.create_subscription(Float32, 'imu/heading_deg', self._on_heading, 10)
        self.create_subscription(String, 'motor/status', self._on_motor, 10)
        self.create_subscription(Bool, 'safety/stop', self._on_stop, 10)
        self.create_subscription(
            Bool, 'safety/emergency_stop', self._on_emergency_stop, 10)
        self.create_subscription(
            CompressedImage, 'camera/image/compressed', self._on_image, 5)
        self.create_timer(0.05, self._tick)

    def _on_action(self, msg):
        try: data = json.loads(msg.data)
        except Exception: return
        action = str(data.get('action', 'idle')).lower()
        self.action = action
        if action == 'approach' and self.phase in ('idle', 'error', 'loaded'):
            # 2호기가 물자로 가지 않는다. 명령을 받는 즉시 180도 회전한 뒤
            # 정지한 채 1호기가 물자를 밀어 오기를 기다린다.
            self.turn_target_deg = float(
                self.get_parameter('turn_target_deg').value)
            # 다음 tick에서 fresh heading으로 회전 원점을 잡는다.
            self.turn_origin = None
            self.turn_next_phase = 'approach'
            self.phase = 'turning'
            self.contact_hits = 0
            self.last_leader_status_update = self.leader_status_update
            self.load_requested = False
            self.loading_started = None
            self.load_high_seen = False
            self.load_drop_since = None
            self.roller_value = 0.0
            self.reason = ''
        elif action == 'turn' and self.phase in ('idle', 'error', 'loaded'):
            self.turn_target_deg = abs(float(data.get('angle_deg', 180.0)))
            self.turn_origin = None
            self.turn_next_phase = 'idle'
            self.phase = 'turning'
        elif action == 'load' and self.phase in (
                'turning', 'approach', 'contact', 'loading'):
            # 접촉 판정보다 명령이 먼저 도착해도 버리지 않고 보류한다.
            self.load_requested = True
            self.roller_value = max(-1.0, min(1.0, float(data.get('roller', 0.0))))
            if self.phase == 'contact':
                self._begin_loading(time.monotonic())
        elif action == 'hold':
            self.roller_value = 0.0
            self.load_requested = False
            if self.phase != 'loaded':
                self._fail('hold_before_load_complete')
        elif action == 'couple':
            # 2026-09-03: 결합 라우팅을 여기서 뺐다. coupling.py 가
            # fleet/action 을 직접 구독한다 — 결합은 겨울에도 쓰는데
            # 여름 노드를 거치면 계절별 게이팅이 생겼을 때 끊긴다
            # (TODO 24번). 이 노드는 화물만 담당한다.
            pass
        elif action == 'release':
            self.roller_value = float(self.get_parameter('release_roller').value)
        elif action in ('idle', 'stop'):
            self.roller_value = 0.0
            self.load_requested = False
            if action == 'stop':
                self.phase = 'idle'

    @staticmethod
    def _drive_current_from_status(status):
        """상태 표시용으로 fleet status에서 주행 모터 전류를 추출한다.

        새 ``drive_current_a``를 우선 사용하고, 1호기가 아직 예전 규약이면
        기존 ``current_a``를 호환용으로 받는다.
        """
        direct = status.get('drive_current_a')
        if isinstance(direct, (int, float)) and math.isfinite(direct):
            return abs(float(direct)), 'drive_current_a'

        drive_ids = {str(value) for value in status.get('drive_motor_ids', [])}
        values = []
        for motor_id, item in status.get('motors', {}).items():
            value = item.get('current_a')
            if (str(motor_id) in drive_ids and
                    item.get('current_fresh', True) and
                    isinstance(value, (int, float)) and
                    math.isfinite(value)):
                values.append(abs(float(value)))
        if values:
            return max(values), 'motors'

        legacy = status.get('current_a')
        if isinstance(legacy, (int, float)) and math.isfinite(legacy):
            return abs(float(legacy)), 'legacy_current_a'
        return None, ''

    def _on_leader_status(self, msg):
        try:
            status = json.loads(msg.data)
            if int(status.get('unit', 1)) != 1:
                return
            now = time.monotonic()
            current, source = self._drive_current_from_status(status)
            self.leader_current_a = current
            self.leader_current_source = source
            # 전류 기준값은 1호기 설정에만 둔다. 2호기는 판정 결과만 받는다.
            self.leader_contact_detected = bool(
                status.get('cargo_contact_detected', False))
            self.leader_load_current_dropped = bool(
                status.get('cargo_load_current_dropped', False))
            self.leader_current_detection_ready = bool(
                status.get('cargo_current_detection_ready', False))
            self.leader_status_time = now
            self.leader_status_update += 1
        except Exception:
            pass

    def _on_roller_manual(self, msg):
        self.manual_roller = max(-1.0, min(1.0, float(msg.data)))
        self.manual_roller_time = time.monotonic()

    def _on_uwb(self, msg): self.uwb_mm, self.uwb_time = float(msg.data), time.monotonic()
    def _on_heading(self, msg): self.heading, self.heading_time = float(msg.data), time.monotonic()
    def _on_stop(self, msg): self.safety_stop = bool(msg.data)
    def _on_emergency_stop(self, msg): self.emergency_stop = bool(msg.data)

    def _on_motor(self, msg):
        try:
            status = json.loads(msg.data)
            drive_ids = status.get('drive_motor_ids', [])
            if isinstance(drive_ids, list) and len(drive_ids) == 2:
                self.drive_motor_ids = [str(value) for value in drive_ids]
            values = []
            for motor_id, item in status.get('motors', {}).items():
                value = item.get('current_a')
                if (str(motor_id) in self.drive_motor_ids and
                        item.get('current_fresh', True) and
                        isinstance(value, (int, float)) and math.isfinite(value)):
                    values.append(abs(float(value)))
            sequence = status.get('current_seq')
            if isinstance(sequence, int) and sequence != self.last_motor_sequence:
                self.last_motor_sequence = sequence
                self.current_update = sequence
                self.current_a = max(values) if values else None
                self.current_time = time.monotonic() if values else None
            elif sequence is None and values:
                self.current_update += 1
                self.current_a = max(values)
                self.current_time = time.monotonic()
            elif not values:
                self.current_a = None
        except Exception: pass

    def _fail(self, reason):
        self.phase = 'error'
        self.reason = reason
        self.contact_hits = 0
        self.load_requested = False
        self.loading_started = None
        self.load_high_seen = False
        self.load_drop_since = None
        self.roller_value = 0.0

    def _load_current_thresholds(self):
        high = float(self.get_parameter(
            'unit2_load_high_current_a').value)
        low = float(self.get_parameter(
            'unit2_load_low_current_a').value)
        return high, low

    def _load_current_threshold_ready(self):
        high, low = self._load_current_thresholds()
        return high > 0.0 and 0.0 < low < high

    def _leader_status_ready(self, now):
        return (self.leader_status_time is not None and
                now - self.leader_status_time <= float(
                    self.get_parameter('sensor_timeout_s').value))

    def _update_load_drop(self, current, now):
        """2호기에서 부하 상승 후 하락이 유지됐는지 확인한다."""
        high, low = self._load_current_thresholds()
        if not (high > 0.0 and 0.0 < low < high):
            self.load_high_seen = False
            self.load_drop_since = None
            return False
        if current >= high:
            self.load_high_seen = True
            self.load_drop_since = None
            return False
        if not self.load_high_seen:
            self.load_drop_since = None
            return False
        if current <= low:
            if self.load_drop_since is None:
                self.load_drop_since = now
            return now - self.load_drop_since >= float(
                self.get_parameter('load_complete_hold_s').value)
        self.load_drop_since = None
        return False

    def _begin_loading(self, now):
        leader_ready = (
            self._leader_status_ready(now) and
            self.leader_current_detection_ready)
        if not self._load_current_threshold_ready() and not leader_ready:
            self._fail('load_current_threshold_unset')
            return False
        self.phase = 'loading'
        self.reason = ''
        self.loading_started = now
        self.load_high_seen = False
        self.load_drop_since = None
        return True

    def _on_image(self, msg):
        # 프레임마다 돌리지 않는다 — red_detect_rate_hz 로 솎는다.
        # 디코딩 자체가 비싸므로 판정 전에 먼저 걸러야 의미가 있다.
        rate = float(self.get_parameter('red_detect_rate_hz').value)
        if rate > 0.0:
            now = time.monotonic()
            if now - self.last_red_detect < 1.0 / rate:
                return
            self.last_red_detect = now
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None: return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        s, v = int(self.get_parameter('red_s_min').value), int(self.get_parameter('red_v_min').value)
        lo1 = np.array([int(self.get_parameter('red_h_min_1').value), s, v])
        hi1 = np.array([int(self.get_parameter('red_h_max_1').value), 255, 255])
        lo2 = np.array([int(self.get_parameter('red_h_min_2').value), s, v])
        hi2 = np.array([int(self.get_parameter('red_h_max_2').value), 255, 255])
        mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area = max((cv2.contourArea(c) for c in contours), default=0.0)
        self.red_detected = area >= float(self.get_parameter('red_min_area').value)

    def _tick(self):
        now = time.monotonic()
        cmd = Twist()
        timeout = float(self.get_parameter('sensor_timeout_s').value)
        uwb_ready = self.uwb_time is not None and now - self.uwb_time <= timeout
        heading_ready = self.heading_time is not None and now - self.heading_time <= timeout
        current_ready = self.current_time is not None and now - self.current_time <= timeout
        leader_status_ready = self._leader_status_ready(now)
        uwb_contact_ready = (
            uwb_ready and self.uwb_mm <= float(
                self.get_parameter('contact_uwb_threshold_mm').value))
        stopped = self.safety_stop or self.emergency_stop
        if stopped and self.phase in (
                'turning', 'approach', 'contact', 'loading'):
            self._fail('safety_stop')
        if not stopped and self.action not in ('idle', 'stop'):
            if self.phase == 'turning':
                if heading_ready and self.turn_origin is None:
                    self.turn_origin = self.heading
                if heading_ready and self.turn_origin is not None:
                    if angle_delta(self.heading, self.turn_origin) >= (
                            self.turn_target_deg -
                            float(self.get_parameter('turn_tolerance_deg').value)):
                        self.phase = self.turn_next_phase
                        self.turn_origin = None
                    else:
                        cmd.angular.z = float(self.get_parameter('turn_speed_rps').value)
            elif self.phase == 'approach':
                # 이 단계에서 2호기는 정지한다. UWB는 전류 접촉 감시를
                # 허용하는 게이트이고, 접촉 자체는 1호기 주행 전류로 본다.
                if not uwb_contact_ready:
                    self.contact_hits = 0
                if self.leader_status_update != self.last_leader_status_update:
                    self.last_leader_status_update = self.leader_status_update
                    if (uwb_contact_ready and leader_status_ready and
                            self.leader_current_detection_ready and
                            self.leader_contact_detected):
                        self.contact_hits += 1
                    else:
                        self.contact_hits = 0
                if self.contact_hits >= int(
                        self.get_parameter('contact_confirm_samples').value):
                    self.phase = 'contact'
                    self.reason = ''
                    if self.load_requested:
                        self._begin_loading(now)
            elif self.phase == 'loading':
                unit2_detection_ready = self._load_current_threshold_ready()
                unit2_detection_usable = unit2_detection_ready and current_ready
                leader_detection_usable = (
                    leader_status_ready and
                    self.leader_current_detection_ready)
                if not unit2_detection_usable and not leader_detection_usable:
                    self._fail('loading_current_detection_lost')
                elif now - self.loading_started > float(
                        self.get_parameter('loading_timeout_s').value):
                    self._fail('loading_timeout')
                else:
                    cmd.linear.x = float(
                        self.get_parameter('load_speed_mps').value)
                    minimum_elapsed = now - self.loading_started >= float(
                        self.get_parameter('load_min_duration_s').value)
                    if unit2_detection_usable:
                        unit2_drop = self._update_load_drop(self.current_a, now)
                    else:
                        self.load_drop_since = None
                        unit2_drop = False
                    leader_drop = (
                        leader_detection_usable and
                        self.leader_load_current_dropped)
                    if minimum_elapsed and (leader_drop or unit2_drop):
                        cmd = Twist()
                        self.phase = 'loaded'
                        self.roller_value = 0.0
                        self.load_requested = False
                        self.reason = ''
        self.cmd_pub.publish(cmd)
        # 자동 미션이 안 도는(idle/stop) 동안만 수동 컨베이어 값을 받는다 —
        # 자동 시퀀스가 시작되면 즉시 자동 쪽이 우선하고, 콘솔이 값 갱신을
        # 멈추면(timeout) 자동으로 0이 된다.
        manual_fresh = (
            self.manual_roller is not None and self.manual_roller_time is not None and
            now - self.manual_roller_time <= float(
                self.get_parameter('manual_roller_timeout_s').value))
        if self.action in ('idle', 'stop') and manual_fresh:
            roller_value = self.manual_roller
        elif self.phase == 'loading' or self.action == 'release':
            roller_value = self.roller_value
        else:
            roller_value = 0.0
        roller = Float32(); roller.data = float(roller_value if not stopped else 0.0)
        self.roller_pub.publish(roller)
        red = Bool(); red.data = self.red_detected; self.red_pub.publish(red)
        status = String(); status.data = json.dumps({
            'phase': self.phase, 'action': self.action, 'red_detected': self.red_detected,
            'uwb_ready': uwb_ready, 'heading_ready': heading_ready,
            'current_ready': current_ready, 'current_a': self.current_a,
            'leader_status_ready': leader_status_ready,
            'leader_drive_current_a': self.leader_current_a,
            'leader_current_source': self.leader_current_source,
            'leader_contact_detected': self.leader_contact_detected,
            'leader_load_current_dropped': self.leader_load_current_dropped,
            'leader_current_detection_ready': self.leader_current_detection_ready,
            'uwb_contact_ready': uwb_contact_ready,
            'contact_uwb_threshold_mm': float(self.get_parameter(
                'contact_uwb_threshold_mm').value),
            'contact_hits': self.contact_hits,
            'unit2_load_current_thresholds': dict(zip(
                ('high_a', 'low_a'), self._load_current_thresholds())),
            'unit2_load_current_threshold_ready': (
                self._load_current_threshold_ready()),
            'unit2_load_high_seen': self.load_high_seen,
            'unit2_load_drop_for_s': (
                0.0 if self.load_drop_since is None else
                max(0.0, now - self.load_drop_since)),
            'load_requested': self.load_requested,
            'safety_stop': self.safety_stop,
            'emergency_stop': self.emergency_stop,
            'reason': self.reason,
            'linear': cmd.linear.x, 'angular': cmd.angular.z, 'roller': roller.data,
            'roller_manual_active': bool(manual_fresh and self.action in ('idle', 'stop')),
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args); node = CargoLoad()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.roller_pub.publish(Float32(data=0.0))
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
