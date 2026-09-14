"""ArUco를 PI 제어로 추적한 뒤 블라인드 접촉과 당김 전류로 결합한다."""
import json
import math
import statistics
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_error(current, target):
    return (float(current) - float(target) + 180.0) % 360.0 - 180.0


class Coupling(Node):
    def __init__(self):
        super().__init__('coupling')

        # 결합 속도와 허용 오차는 mission/config/coupling.yaml에서 조정한다.
        parameters = {
            'marker_confirm_s': 1.0,
            # 기준 자세를 저장하기 전에 마커가 안정돼야 하는 시간.
            # 0.0 이면 처음 보이는 프레임에서 바로 저장한다 — 기준 자세는
            # 이제 제어에 안 쓰이고(정렬은 화면 중심 오차만 본다) 상태
            # 진단용이라 대기시킬 이유가 없다. 예전처럼 안정화를
            # 기다리게 하려면 marker_confirm_s 와 같은 값을 넣는다.
            'reference_hold_s': 0.0,
            'marker_release_s': 1.0,
            # Reference acquisition must survive brief detector flicker.
            'marker_acquire_dropout_s': 0.35,
            'marker_control_timeout_s': 0.35,
            'align_timeout_s': 15.0,
            # 초기 정렬이 이 시간 유지되면 contact로 넘어간다. contact에서도
            # 마커가 보이는 동안은 PI로 중심을 유지하며 전진한다.
            'initial_align_confirm_s': 0.4,
            # 보이는 동안에는 이 크기까지 접근 속도를 연속 감속한다.
            'marker_blind_arm_side_px': 220.0,
            # Blind contact is armed only after a real increasing-size trend.
            'marker_blind_min_side_px': 120.0,
            'marker_growth_window_s': 0.8,
            'marker_growth_min_duration_s': 0.35,
            'marker_growth_min_px': 20.0,
            'marker_growth_min_ratio': 1.15,
            'marker_growth_positive_ratio': 0.65,
            # marker_vision already confirms loss from a multi-frame window.
            'marker_disappear_confirm_s': 0.0,
            'marker_approach_speed_mps': 0.10,
            'marker_near_speed_mps': 0.04,
            # 마커 중심의 가로 오차를 마커 한 변 길이로 나눈 비율.
            # 거리(z)는 정렬 완료 조건에 사용하지 않는다.
            'lateral_tolerance_marker_ratio': 0.08,
            # 2026-09-01 재작성. 7월 docking_ctrl.py 구조로 되돌린다.
            # 오차를 '방위각'(마커까지의 좌우 각도)으로 쓴다 —
            #   bearing = atan2(x_mm, z_mm)
            # 화면 중심비(offset_px/side_px)는 거리가 바뀌면 같은 어긋남도
            # 값이 달라져 게인이 거리에 딸려 변한다. 방위각은 그 왜곡이 없다.
            'bearing_tolerance_deg': 2.0,
            'kp_bearing_rps_per_rad': 0.6,
            # 완료는 시간이 아니라 **새 마커 표본 연속 N회**로 센다.
            # 진동 중에는 한 번만 튀어도 0 으로 리셋돼 통과하지 못한다.
            'align_stable_samples': 6,
            'lateral_tolerance_m': 0.01,
            'roll_tolerance_deg': 2.0,
            'pitch_tolerance_deg': 4.0,
            'yaw_tolerance_deg': 2.0,
            # solvePnP z_mm은 멀수록 잡음이 커진다(60cm서 약 7%,
            # base/marker_vision.md 참고) — 이 계수로 지수이동평균해
            # 정렬 판정에 쓴다.
            'z_ema_alpha': 0.35,
            'k_lateral': 2.0,
            'ki_lateral': 0.35,
            # 감쇠항. 중심 오차가 줄어드는 '속도'를 빼서 관성으로 중심을
            # 지나치는 것을 막는다. 2026-09-01 실측에서 명령이 0.03 rad/s
            # 로 작아진 뒤에도 0.4초 만에 중심비가 0.73 움직였다 —
            # 명령으로 설명되는 0.155 의 5배라, 남은 건 관성이다.
            'kd_lateral': 0.15,
            'k_yaw': 0.012,
            'ki_yaw': 0.002,
            'lateral_integral_limit_m_s': 0.08,
            'yaw_integral_limit_deg_s': 30.0,
            'k_roll': 0.0,
            'k_pitch': 0.0,
            'max_forward_mps': 0.25,
            'max_reverse_mps': 0.25,
            'max_angular_rps': 0.45,
            'angular_pause_deg': 10.0,
            'contact_speed_mps': 0.10,
            # 이 단계 전용 접촉 전류 기준. 0 이면 motor/status 의 공통값을
            # 쓴다. 결합과 적재는 접근 속도가 달라 접촉 순간의 전류도
            # 다르므로(속도가 빠를수록 크게 튄다) 값을 따로 갖는다.
            'contact_current_threshold_a': 0.0,
            'contact_current_hold_s': 0.5,
            'current_timeout_s': 0.5,
            # contact의 ArUco 추적 구간과 마커 소실 후 블라인드 구간을
            # 따로 제한한다. contact_timeout_s는 블라인드 직진에만 적용된다.
            'contact_visual_timeout_s': 15.0,
            'blind_contact_distance_m': 0.30,
            'contact_timeout_s': 4.0,
            'solenoid_timeout_s': 1.5,
            # 잠금 확인 뒤 2호기가 뒤로 당겨 실제 결합 하중을 검증한다.
            'verify_pull_speed_mps': -0.08,
            # 0이면 contact_current_threshold_a를 그대로 쓴다.
            'verify_current_threshold_a': 0.0,
            'verify_current_confirm_samples': 3,
            # 검증 실패 시 재결합을 몇 번까지 시도할지. 넘으면 멈추고
            # 사람을 부른다 — 그래도 솔레노이드는 풀지 않는다(TODO 21번).
            'verify_retry_limit': 3,
            # -0.08 m/s 기준 실패 시 최대 약 8 cm만 후진한다.
            'verify_timeout_s': 1.0,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self.reference = None
        self.z_ema_mm = None
        self.marker_samples = deque(maxlen=50)
        # 마커 표본 번호. 제어는 **새 표본이 왔을 때만** 다시 계산한다.
        # 제어 tick(20Hz)이 마커(11Hz)보다 빨라서, 같은 값에 두 번씩
        # 반응하면 적분이 배로 쌓이고 미분이 0 과 급변을 오간다.
        self.marker_seq = 0
        self.processed_seq = -1
        self.last_align_cmd = Twist()
        self.aligned_samples = 0
        self.marker_visible = False
        # marker_visible은 여러 프레임 필터 결과, marker_raw_visible은
        # 현재 원본 프레임의 실제 검출 여부다. 제어에는 오래된 pose를
        # 재사용하지 않도록 raw를 함께 확인한다.
        self.marker_raw_visible = False
        self.marker_stable = False
        self.marker_seen_since = None
        self.marker_lost_since = None
        self.latest_marker = None
        self.latest_marker_time = None
        self.marker_status_time = None
        # Backward-compatible optimistic default; a real marker/status message
        # immediately replaces it with camera_frame_fresh.
        self.marker_camera_fresh = True
        self.marker_detection_enabled = True
        self.motor_can = False
        self.current_a = None
        self.shared_threshold_a = 0.0        # motor/status 가 주는 공통값(폴백)
        self.current_time = None
        self.motor_update = 0
        self.last_motor_sequence = None
        self.last_current_update = -1
        self.solenoid_locked = False
        self.safety_stop = False
        self.emergency_stop = False
        self.active = False
        self.start_pending = False
        self.phase = 'ready'
        self.reason = ''
        self.align_started = None
        self.aligned_since = None
        self.contact_started = None
        self.blind_contact_started = None
        self.contact_blind = False
        self.contact_marker_near = False
        self.contact_start_side_px = None
        self.marker_side_history = deque(maxlen=50)
        self.marker_growth_armed = False
        self.marker_growth = {}
        self.blind_contact_distance_m = 0.0
        self.blind_last_tick = None
        self.contact_current_since = None
        self.lock_started = None
        self.verify_started = None
        self.verify_current_hits = 0
        self.verify_retries = 0
        self.errors = {}
        self.pi_last_time = None
        self.lateral_integral = 0.0
        self.lateral_prev = None
        self.lateral_prev_time = None
        self.lateral_rate = 0.0
        self.yaw_integral = 0.0
        self.output = Twist()

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/coupling', 10)
        self.solenoid_pub = self.create_publisher(Bool, 'solenoid_cmd', 10)
        self.marker_enable_pub = self.create_publisher(
            Bool, 'marker/enable', 10)
        self.status_pub = self.create_publisher(
            String, 'mission/coupling/status', 10)
        self.create_subscription(String, 'marker/status', self._on_marker, 10)
        self.create_subscription(String, 'motor/status', self._on_motor, 10)
        self.create_subscription(String, 'coupling/cmd', self._on_command, 10)
        # 1호기의 couple 액션을 직접 받는다. 예전에는 여름 노드
        # cargo_load 가 받아서 coupling/cmd 로 넘겨줬는데, 결합은
        # 겨울에도 쓰므로 여름 노드를 거치면 안 된다(TODO 24번).
        self.create_subscription(
            String, 'fleet/action', self._on_fleet_action, 10)
        self.create_subscription(Bool, 'solenoid_status', self._on_solenoid, 10)
        self.create_subscription(Bool, 'safety/stop', self._on_safety, 10)
        self.create_subscription(
            Bool, 'safety/emergency_stop', self._on_emergency, 10)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.2, self._publish_status)

    def _parameter(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _marker_values(value):
        # ArUco pose(solvePnP) 필드명 — marker_vision._publish_status()가
        # 선택된 마커의 pose를 이 키들로 최상위에 병합해 내보낸다. pose가
        # 없으면(캘리브레이션 해상도 불일치 등) 'x_mm'이 아예 없다.
        names = (
            'x_mm', 'y_mm', 'z_mm', 'roll_deg', 'pitch_deg', 'yaw_deg',
            'offset_x_px', 'side_px')
        if not value.get('detected') or 'x_mm' not in value:
            return None
        try:
            result = {name: float(value[name]) for name in names}
        except (KeyError, TypeError, ValueError):
            return None
        return result if all(math.isfinite(item) for item in result.values()) else None

    def _on_marker(self, msg):
        now = time.monotonic()
        try:
            status = json.loads(msg.data)
            values = self._marker_values(status)
            status_valid = isinstance(status, dict)
        except Exception:
            values = None
            status = {}
            status_valid = False

        if status_valid:
            self.marker_status_time = now
            self.marker_camera_fresh = bool(
                status.get('camera_frame_fresh', True))
            self.marker_detection_enabled = bool(
                status.get('enabled', self.marker_detection_enabled))
            self.marker_raw_visible = bool(
                status.get('raw_detected', values is not None))
        else:
            self.marker_raw_visible = False

        if values is not None:
            # z_mm(거리)만 멀수록 잡음이 커진다(60cm서 약 7%) — 지수이동
            # 평균으로 완화해서 정렬 판정에 쓴다. x/y/roll/pitch/yaw는
            # 이미 안정적이라 그대로 둔다.
            alpha = clamp(float(self._parameter('z_ema_alpha')), 0.0, 1.0)
            self.z_ema_mm = (values['z_mm'] if self.z_ema_mm is None else
                              (1.0 - alpha) * self.z_ema_mm + alpha * values['z_mm'])
            values = dict(values, z_mm=self.z_ema_mm)

        self.marker_visible = values is not None
        if values is not None:
            self.latest_marker = values
            self.latest_marker_time = now
            self.marker_seq += 1
            self.marker_lost_since = None
            if self.marker_seen_since is None:
                self.marker_seen_since = now
                self.marker_samples.clear()
            self.marker_samples.append((now, values))
            if (self.reference is None and
                    now - self.marker_seen_since >= float(
                        self._parameter('reference_hold_s'))):
                self._capture_reference(now)
            if now - self.marker_seen_since >= float(self._parameter('marker_confirm_s')):
                self.marker_stable = True
        else:
            if self.marker_lost_since is None:
                self.marker_lost_since = now
            lost_for = now - self.marker_lost_since
            if (not self.marker_stable and
                    lost_for >= float(
                        self._parameter('marker_acquire_dropout_s'))):
                self.marker_seen_since = None
                self.marker_samples.clear()
                self.z_ema_mm = None
            elif (self.marker_stable and
                    lost_for >= float(
                        self._parameter('marker_release_s'))):
                self.marker_stable = False
                self.marker_seen_since = None
                self.marker_samples.clear()
                self.z_ema_mm = None
                if not self.active and self.phase != 'locked':
                    self.phase = 'marker_wait'

    def _capture_reference(self, now):
        window = float(self._parameter('marker_confirm_s'))
        samples = [values for stamp, values in self.marker_samples
                   if now - stamp <= window + 0.25]
        if not samples:
            return
        self.reference = {
            name: float(statistics.median(item[name] for item in samples))
            for name in ('x_mm', 'y_mm', 'z_mm', 'roll_deg', 'pitch_deg', 'yaw_deg')
        }
        self.phase = 'ready'
        self.reason = ''
        self.get_logger().info(
            'Coupling reference captured: ' + json.dumps(self.reference))

    def _on_motor(self, msg):
        try:
            value = json.loads(msg.data)
            currents = []
            # 예전에는 ID '1','2' 를 박아뒀다. motor.yaml 에서 left_id/right_id 를
            # 바꾸면 조용히 전류를 못 읽어 결합이 시작조차 안 된다.
            # motor.py 는 주행축에만 Iq 를 요청하므로 롤러 전류는 애초에 안 온다.
            for motor in value.get('motors', {}).values():
                current = motor.get('current_a')
                if (motor.get('current_fresh', True) and
                        isinstance(current, (int, float)) and math.isfinite(current)):
                    currents.append(abs(float(current)))
            self.motor_can = bool(value.get('can_connected', False))
            threshold = value.get('contact_current_threshold_a', 0.0)
            self.shared_threshold_a = (
                float(threshold) if isinstance(threshold, (int, float)) else 0.0)
            sequence = value.get('current_seq')
            if isinstance(sequence, int) and sequence != self.last_motor_sequence:
                self.last_motor_sequence = sequence
                self.motor_update = sequence
                self.current_a = max(currents) if currents else None
                self.current_time = time.monotonic() if currents else None
            elif sequence is None and currents:
                self.motor_update += 1
                self.current_a = max(currents)
                self.current_time = time.monotonic()
            elif not currents:
                self.current_a = None
        except Exception:
            pass

    def _on_solenoid(self, msg):
        self.solenoid_locked = bool(msg.data)

    def _on_safety(self, msg):
        self.safety_stop = bool(msg.data)

    def _on_emergency(self, msg):
        self.emergency_stop = bool(msg.data)

    def _on_fleet_action(self, msg):
        """1호기 액션 중 couple 만 본다 — 나머지는 다른 노드 몫이다."""
        try:
            action = str(json.loads(msg.data).get('action', '')).lower()
        except Exception:
            action = str(msg.data).strip().lower()
        if action == 'couple':
            self._publish_marker_enabled(True)
            self.start_pending = True
            self.verify_retries = 0
            self._try_start()

    def _on_command(self, msg):
        try:
            value = json.loads(msg.data)
            action = str(value.get('action', '')).lower()
        except Exception:
            action = str(msg.data).strip().lower()

        if action == 'start':
            # 준비 전 들어온 명령도 버리지 않는다. _tick()이 eligibility를
            # 계속 확인하고 조건이 갖춰지는 순간 한 번만 시작한다.
            self._publish_marker_enabled(True)
            self.start_pending = True
            self.verify_retries = 0
            self._try_start()
        elif action == 'cancel':
            # 진행 중인 결합만 멈춘다. 솔레노이드는 건드리지 않는다 —
            # 예전에는 여기서도 풀어서 취소할 때마다 잠금이 풀렸다.
            self.start_pending = False
            self.verify_retries = 0
            self._abort('cancelled')
            self.phase = 'ready'
        elif action == 'unlock':
            # 해제는 사람이 명시적으로 누를 때만 하는 유일한 경로다.
            self.start_pending = False
            self.verify_retries = 0
            self._abort('unlocked', unlock=True)
            self.phase = 'ready'
        elif action == 'recapture' and not self.active:
            self.start_pending = False
            self._publish_marker_enabled(True)
            self.reference = None
            self.marker_stable = False
            self.marker_seen_since = None
            self.marker_samples.clear()
            self.z_ema_mm = None
            self.phase = 'ready'

    def _try_start(self):
        if not self.start_pending:
            return False
        eligible, reason = self._eligibility()
        if not eligible:
            self.reason = reason
            # 이미 진행 중이거나 성공한 결합은 재시도 대상으로 남기지 않는다.
            if reason in ('active', 'locked'):
                self.start_pending = False
            return False
        # 기준 자세 수집을 기다리지 않는다. 시작 순간의 현재 pose는
        # 제어 조건이 아니라 상태 진단용 기준으로만 즉시 저장한다.
        self.reference = {
            name: float(self.latest_marker[name])
            for name in (
                'x_mm', 'y_mm', 'z_mm',
                'roll_deg', 'pitch_deg', 'yaw_deg')
        }
        self.start_pending = False
        self.active = True
        self.phase = 'align'
        self.reason = ''
        self.align_started = time.monotonic()
        self.aligned_since = None
        self.contact_started = None
        self.blind_contact_started = None
        self.contact_blind = False
        self.contact_marker_near = False
        self._reset_marker_growth()
        self.blind_contact_distance_m = 0.0
        self.blind_last_tick = None
        self.contact_current_since = None
        self._reset_pi()
        # 대기 중 걸쇠는 잠긴 상태가 정상이다. 접근 전에 해제하지 않고,
        # 접촉 후 locking 단계에서 잠금 명령을 다시 보내 상태를 확정한다.
        return True

    def _current_ready(self):
        return (self.current_a is not None and self.current_time is not None and
                time.monotonic() - self.current_time <= float(
                    self._parameter('current_timeout_s')))

    def contact_threshold(self):
        """이 단계에 적용할 접촉 전류 기준(A).

        자기 설정이 양수면 그것을, 아니면 ``motor/status`` 의 공통값을 쓴다.
        둘 다 0 이면 접촉 판정을 하지 않는다(실측 전 안전 상태).
        """
        own = float(self._parameter('contact_current_threshold_a'))
        return own if own > 0.0 else self.shared_threshold_a

    def verify_current_threshold(self):
        """후진 당김 검증용 전류 기준. 별도 값이 없으면 접촉 기준을 쓴다."""
        own = float(self._parameter('verify_current_threshold_a'))
        return own if own > 0.0 else self.contact_threshold()

    def _eligibility(self):
        if self.active:
            return False, 'active'
        if self.phase == 'locked':
            return False, 'locked'
        if self.contact_threshold() <= 0.0:
            return False, 'current_threshold_unset'
        if not self._marker_status_fresh():
            return False, 'marker_status_wait'
        if not self.marker_camera_fresh:
            return False, 'camera_frame_wait'
        if (not self.marker_raw_visible or not self.marker_visible or
                not self._marker_fresh()):
            return False, 'marker_wait'
        if not self.motor_can:
            return False, 'can_unavailable'
        if not self.solenoid_locked:
            return False, 'solenoid_unlocked'
        if self.safety_stop or self.emergency_stop:
            return False, 'safety_stop'
        return True, 'ready'

    def _marker_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.latest_marker is not None and self.latest_marker_time is not None and
                now - self.latest_marker_time <= float(
                    self._parameter('marker_control_timeout_s')))

    def _marker_status_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.marker_status_time is not None and
                now - self.marker_status_time <= float(
                    self._parameter('marker_control_timeout_s')))

    def _update_alignment_errors(self):
        """기준 자세와 현재 ArUco pose의 오차를 갱신한다."""
        reference = self.reference
        current = self.latest_marker
        distance_error = (current['z_mm'] - reference['z_mm']) / 1000.0
        lateral_error = (current['x_mm'] - reference['x_mm']) / 1000.0
        roll_error = angle_error(current['roll_deg'], reference['roll_deg'])
        pitch_error = angle_error(current['pitch_deg'], reference['pitch_deg'])
        yaw_error = angle_error(current['yaw_deg'], reference['yaw_deg'])
        side_px = float(current['side_px'])
        # 화면 가로 중심 오차를 현재 마커 크기로 정규화한다. 로봇이
        # 가까워져 마커가 커져도 앞뒤 거리와 무관하게 좌우만 맞춘다.
        lateral_marker_ratio = float(current['offset_x_px']) / max(1.0, side_px)
        # 방위각: 마커가 정면에서 좌우로 몇 도 벗어나 있나.
        # z 가 0 에 가까워도 발산하지 않게 하한을 둔다(7월 코드와 동일).
        bearing_deg = math.degrees(math.atan2(
            current['x_mm'], max(50.0, current['z_mm'])))
        side_limit = float(self._parameter('marker_blind_arm_side_px'))
        self.errors = {
            'distance_m': distance_error,
            'lateral_m': lateral_error,
            'lateral_marker_ratio': lateral_marker_ratio,
            'bearing_deg': bearing_deg,
            'roll_deg': roll_error,
            'pitch_deg': pitch_error,
            'yaw_deg': yaw_error,
            'marker_side_px': side_px,
            'marker_blind_arm_side_px': side_limit,
        }
        return self.errors

    def _initial_aligned(self):
        """contact 전 초기 정렬 완료 여부.

        방위각이 허용치 안에 든 **새 마커 표본이 연속 N 개** 쌓여야 한다.
        시간 기준(예: 0.4초 유지)은 20Hz tick 이 같은 표본을 여러 번 세는
        바람에 진동 중에도 통과할 수 있었다. 표본 기준이면 한 번만 벗어나도
        0 으로 리셋된다.

        앞뒤 거리(z)와 roll/pitch/yaw 는 상태에만 남기고 조건으로 쓰지 않는다.
        스키드스티어는 조향 입력이 하나뿐이라 방위각과 마커 yaw 를 동시에
        0 으로 만들 수 없다(2026-09-01 실측에서 두 항이 서로 밀어냈다).
        """
        self._update_alignment_errors()
        return self.aligned_samples >= int(
            self._parameter('align_stable_samples'))

    def _reset_pi(self):
        self.pi_last_time = None
        # 다음 tick 이 반드시 새로 계산하도록 표본 번호를 비운다.
        self.processed_seq = -1
        self.aligned_samples = 0
        self.last_align_cmd = Twist()
        self.lateral_integral = 0.0
        self.lateral_prev = None
        self.lateral_prev_time = None
        self.lateral_rate = 0.0
        self.yaw_integral = 0.0

    def _alignment_command(self, now, forward_speed, forward_limit=None):
        """방위각 P 제어. 새 마커 표본이 왔을 때만 다시 계산한다.

        2026-09-01 재작성. 7월 ``robot_control/docking_ctrl.py`` 구조로 되돌린
        것이고, 4색 마커 대신 ArUco pose 를 쓴다(``x_mm``/``z_mm`` 이 그대로
        lateral/distance 에 대응한다).

        바뀐 점 세 가지와 그 이유:

        1. **새 표본에만 반응한다.** 제어 tick 은 20Hz 인데 마커는 11Hz 라,
           같은 값에 두 번씩 반응하면 적분이 배로 쌓이고 미분이 0 과 급변을
           오간다. 표본이 안 바뀌면 직전 조향을 그대로 유지한다.
        2. **오차를 방위각으로 쓴다.** 화면 중심비(offset_px/side_px)는 거리가
           바뀌면 같은 어긋남도 값이 달라져 게인이 거리에 딸려 변한다.
           방위각 atan2(x_mm, z_mm) 은 그 왜곡이 없다.
        3. **허용치 안에서는 조향을 0 으로 끊는다.** 잔여 오차를 계속 쫓으면
           관성과 겹쳐 넘어간다(2026-09-01 실측: ±0.5 중심비 limit cycle).

        마커 yaw 는 조향에 넣지 않는다. 스키드스티어는 조향 입력이 하나뿐이라
        방위각과 마커 yaw 를 동시에 0 으로 만들 수 없고, 둘을 같이 넣었을 때
        서로 반대 명령을 내며 좌우로 흔들렸다.
        """
        if forward_limit is None:
            forward_limit = float(self._parameter('max_forward_mps'))
        linear = clamp(
            abs(float(forward_speed)), 0.0, abs(float(forward_limit)))

        if self.marker_seq == self.processed_seq:
            # 새 표본 없음 — 직전 조향을 유지한다(재계산하지 않는다).
            hold = Twist()
            hold.linear.x = float(linear)
            hold.angular.z = float(self.last_align_cmd.angular.z)
            return hold
        self.processed_seq = self.marker_seq
        self.pi_last_time = now

        errors = self._update_alignment_errors()
        bearing_deg = float(errors['bearing_deg'])
        tolerance = float(self._parameter('bearing_tolerance_deg'))
        limit = float(self._parameter('max_angular_rps'))

        if abs(bearing_deg) <= tolerance:
            self.aligned_samples += 1
            angular = 0.0
        else:
            self.aligned_samples = 0
            angular = clamp(
                -float(self._parameter('kp_bearing_rps_per_rad')) *
                math.radians(bearing_deg), -limit, limit)

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.last_align_cmd = cmd
        return cmd

    def _enter_contact(self, now):
        self.phase = 'contact'
        self.contact_started = now
        self.blind_contact_started = None
        self.contact_blind = False
        self.contact_marker_near = False
        self._reset_marker_growth()
        if self.latest_marker is not None:
            self._record_marker_growth(now)
        self.blind_contact_distance_m = 0.0
        self.blind_last_tick = None
        self.contact_current_since = None
        self.last_current_update = self.motor_update
        self._reset_pi()

    def _reset_marker_growth(self):
        self.contact_start_side_px = None
        self.marker_side_history.clear()
        self.marker_growth_armed = False
        self.marker_growth = {}

    def _record_marker_growth(self, now):
        """Arm blind contact only from a sustained increasing marker size."""
        if self.latest_marker is None:
            return False
        side_px = float(self.latest_marker['side_px'])
        if self.contact_start_side_px is None:
            self.contact_start_side_px = side_px
        self.contact_marker_near = (
            side_px >= float(self._parameter('marker_blind_arm_side_px')))
        self.marker_side_history.append((float(now), side_px))
        window = float(self._parameter('marker_growth_window_s'))
        cutoff = float(now) - window
        while (self.marker_side_history and
               self.marker_side_history[0][0] < cutoff):
            self.marker_side_history.popleft()

        values = [value for _, value in self.marker_side_history]
        duration = (
            0.0 if len(self.marker_side_history) < 2 else
            self.marker_side_history[-1][0] -
            self.marker_side_history[0][0])
        if len(values) < 5:
            self.marker_growth = {
                'samples': len(values), 'duration_s': duration,
                'armed': self.marker_growth_armed,
            }
            return self.marker_growth_armed

        group = max(2, len(values) // 3)
        first_side = float(statistics.median(values[:group]))
        last_side = float(statistics.median(values[-group:]))
        growth_px = last_side - first_side
        growth_ratio = last_side / max(1.0, first_side)
        nondecreasing = sum(
            1 for previous, current in zip(values, values[1:])
            if current >= previous - 2.0)
        positive_ratio = nondecreasing / max(1, len(values) - 1)
        near_enough = last_side >= float(
            self._parameter('marker_blind_min_side_px'))
        trend_ready = (
            duration >= float(
                self._parameter('marker_growth_min_duration_s')) and
            growth_px >= float(self._parameter('marker_growth_min_px')) and
            growth_ratio >= float(
                self._parameter('marker_growth_min_ratio')) and
            positive_ratio >= float(
                self._parameter('marker_growth_positive_ratio')))
        if near_enough and trend_ready:
            self.marker_growth_armed = True
        self.marker_growth = {
            'samples': len(values),
            'duration_s': duration,
            'first_side_px': first_side,
            'last_side_px': last_side,
            'growth_px': growth_px,
            'growth_ratio': growth_ratio,
            'nondecreasing_ratio': positive_ratio,
            'near_enough': near_enough,
            'trend_ready': trend_ready,
            'armed': self.marker_growth_armed,
        }
        return self.marker_growth_armed

    def _visible_contact_speed(self):
        """Slow down continuously as the visible marker grows."""
        far_speed = abs(float(
            self._parameter('marker_approach_speed_mps')))
        near_speed = min(far_speed, abs(float(
            self._parameter('marker_near_speed_mps'))))
        if self.latest_marker is None:
            return 0.0
        start_side = max(
            1.0, float(self.contact_start_side_px or
                       self.latest_marker['side_px']))
        end_side = max(
            start_side + 1.0,
            float(self._parameter('marker_blind_arm_side_px')))
        progress = clamp(
            (float(self.latest_marker['side_px']) - start_side) /
            (end_side - start_side), 0.0, 1.0)
        return far_speed + (near_speed - far_speed) * progress

    def _enter_blind_contact(self, now):
        """Start straight contact search after confirmed marker loss."""
        self.contact_blind = True
        self.blind_contact_started = now
        self.blind_contact_distance_m = 0.0
        self.blind_last_tick = now
        self._reset_pi()
        self._publish_marker_enabled(False)

    def _verify_retry(self, reason):
        """결합 검증 실패 — 솔레노이드는 그대로 두고 다시 결합한다.

        검증이 실패했다는 것은 '덜 붙었다'는 뜻이지 '풀어야 한다'는
        뜻이 아니다. 잠금은 기계적으로 유지되므로 건드리지 않고
        정렬부터 다시 돌린다. 반복해도 안 붙으면 사람이 봐야 하니
        멈추되, 그때도 해제하지 않는다(2026-09-03 확정, TODO 21번).
        """
        self.verify_retries += 1
        if self.verify_retries > int(
                self._parameter('verify_retry_limit')):
            self._abort(reason)
            return
        self._schedule_marker_retry(reason)

    def _schedule_marker_retry(self, reason):
        """Stop safely and retry automatically when the marker returns."""
        self.active = False
        self.start_pending = True
        self.phase = 'marker_wait'
        self.reason = reason
        self.output = Twist()
        self.aligned_since = None
        self.contact_started = None
        self.blind_contact_started = None
        self.contact_blind = False
        self.contact_marker_near = False
        self._reset_marker_growth()
        self.blind_contact_distance_m = 0.0
        self.blind_last_tick = None
        self.contact_current_since = None
        self._reset_pi()
        self._publish_marker_enabled(True)
        self.marker_stable = False
        self.marker_seen_since = None
        self.marker_samples.clear()
        self.z_ema_mm = None
        self.cmd_pub.publish(self.output)

    def _contact_current_confirmed(self, now):
        """새 전류 표본이 기준 이상으로 0.5초 연속 유지됐는지 확인한다."""
        if self.motor_update == self.last_current_update:
            return False
        self.last_current_update = self.motor_update
        if self.current_a >= self.contact_threshold():
            if self.contact_current_since is None:
                self.contact_current_since = now
            return now - self.contact_current_since >= float(
                self._parameter('contact_current_hold_s'))
        self.contact_current_since = None
        return False

    def _publish_marker_enabled(self, enabled):
        enabled = bool(enabled)
        was_enabled = self.marker_detection_enabled
        self.marker_enable_pub.publish(Bool(data=enabled))
        self.marker_detection_enabled = enabled
        if enabled and not was_enabled:
            # 비활성화 전의 stable/최종 pose를 새 프레임으로 오인하지 않는다.
            self.marker_stable = False
            self.marker_seen_since = None
            self.marker_lost_since = None
            self.marker_samples.clear()
            self.z_ema_mm = None

    def _publish_solenoid(self, locked):
        msg = Bool()
        msg.data = bool(locked)
        self.solenoid_pub.publish(msg)

    def _abort(self, reason, unlock=False):
        self.active = False
        self.start_pending = False
        self.reason = reason
        self.phase = 'error' if reason != 'cancelled' else 'ready'
        self.output = Twist()
        self.aligned_since = None
        self.contact_started = None
        self.blind_contact_started = None
        self.contact_blind = False
        self.contact_marker_near = False
        self._reset_marker_growth()
        self.blind_contact_distance_m = 0.0
        self.blind_last_tick = None
        self.contact_current_since = None
        self.verify_started = None
        self.verify_current_hits = 0
        self._reset_pi()
        self._publish_marker_enabled(True)
        self.cmd_pub.publish(self.output)
        if unlock:
            self._publish_solenoid(False)

    def _tick(self):
        now = time.monotonic()
        cmd = Twist()
        if self.start_pending and not self.active:
            self._try_start()
        if self.active and (self.safety_stop or self.emergency_stop):
            self._abort('safety_stop')
        elif self.active and self.phase == 'align':
            if not self._marker_status_fresh(now):
                # A dead vision stream is held, not converted into a mission
                # failure.  The same active request resumes when it returns.
                self.reason = 'marker_status_wait'
                self.align_started = now
                self.aligned_since = None
                self._reset_pi()
            elif not self.marker_camera_fresh:
                self.reason = 'camera_frame_wait'
                self.align_started = now
                self.aligned_since = None
                self._reset_pi()
            elif (not self.marker_raw_visible or not self.marker_visible or
                  not self._marker_fresh(now)):
                self.reason = 'marker_reacquiring'
                self.align_started = now
                lost_for = (
                    0.0 if self.marker_lost_since is None else
                    max(0.0, now - self.marker_lost_since))
                # Do not throw away the 0.4 s alignment confirmation for an
                # isolated detector miss. A sustained loss still resets it.
                if lost_for >= float(
                        self._parameter('marker_acquire_dropout_s')):
                    self.aligned_since = None
                    self._reset_pi()
            elif now - self.align_started > float(self._parameter('align_timeout_s')):
                self._schedule_marker_retry('align_timeout_retry')
            else:
                self.reason = ''
                aligned = self._initial_aligned()
                if aligned:
                    if self.aligned_since is None:
                        self.aligned_since = now
                    # 완료 확인 중에도 PI로 자세를 유지하되 아직 전진하지 않는다.
                    cmd = self._alignment_command(now, 0.0)
                    if now - self.aligned_since >= float(
                            self._parameter('initial_align_confirm_s')):
                        # contact에서도 마커를 끄지 않고, 중심을 PI로 유지하며
                        # 전진한다. 마커가 실제로 화면 밖으로 나간 뒤에만 블라인드로 넘어간다.
                        self._enter_contact(now)
                        speed = self._visible_contact_speed()
                        cmd = self._alignment_command(
                            now, speed, speed)
                else:
                    self.aligned_since = None
                    # 먼저 좌우 중심과 yaw를 맞춘 뒤 contact에서 접근한다.
                    cmd = self._alignment_command(now, 0.0)
        elif self.active and self.phase == 'contact':
            if not self._current_ready():
                # 전류 없이 접촉 직진은 하지 않는다. 다만 결합 전체를 실패로
                # 끝내지 않고 현재 위치에서 멈춰, 전류가 들어오는 즉시 이어간다.
                self.reason = 'current_unavailable'
                if self.contact_blind:
                    self.blind_contact_started = now
                    self.blind_last_tick = now
                else:
                    self.contact_started = now
            elif self._contact_current_confirmed(now):
                # 블라인드 여부와 무관하게 접촉 전류를 본다.
                #
                # 예전에는 이 판정이 블라인드 구간에만 있었다. 그래서 "가까워
                # 지면 마커가 화면 밖으로 나간다"는 가정이 깨지면 접촉을 아예
                # 못 봤다. 2026-09-01 실측 기하로 그 가정이 성립하지 않는 것이
                # 확인됐다 — fx 1453 px, 마커 40 mm 이면 side_px = 58120/z_mm
                # 이라 마커가 화면(720 px)을 채우는 거리가 약 81 mm 다. 결합봉이
                # 그보다 먼저 닿으므로 마커는 끝까지 보인 채 접촉하게 되고,
                # 그러면 contact_visual_timeout_s 까지 계속 밀었다.
                #
                # 전환 시점(마커 소실/시간/거리)은 어차피 상한일 뿐이고 실제
                # 판정은 전류가 한다. 그래서 전환 시점에 의존하지 않도록 두
                # 구간 모두에서 같은 기준으로 본다. 마커가 실제로 사라지면
                # 기존대로 블라인드로 넘어가고, 거기서도 이 판정을 쓴다.
                self.reason = ''
                cmd = Twist()
                self.phase = 'locking'
                self.lock_started = now
                self._reset_pi()
                self._publish_solenoid(True)
            elif self.contact_blind:
                self.reason = ''
                speed = abs(float(self._parameter('contact_speed_mps')))
                if self.blind_last_tick is None:
                    self.blind_last_tick = now
                dt = clamp(now - self.blind_last_tick, 0.0, 0.1)
                self.blind_last_tick = now
                # Once contact current rises, stop consuming the free-travel
                # budget and allow the 0.5 s contact confirmation to finish.
                if self.current_a < self.contact_threshold():
                    self.blind_contact_distance_m += speed * dt
                if self.blind_contact_distance_m >= float(
                        self._parameter('blind_contact_distance_m')):
                    self._schedule_marker_retry('blind_distance_limit_retry')
                elif now - self.blind_contact_started > float(
                        self._parameter('contact_timeout_s')):
                    self._schedule_marker_retry('contact_timeout_retry')
                else:
                    cmd.linear.x = speed
            elif now - self.contact_started > float(
                    self._parameter('contact_visual_timeout_s')):
                self._schedule_marker_retry('marker_approach_timeout_retry')
            elif not self._marker_status_fresh(now):
                self.reason = 'marker_status_wait'
                self.contact_started = now
                self._reset_pi()
            elif not self.marker_camera_fresh:
                # A stopped camera is not a marker leaving the field of view.
                self.reason = 'camera_frame_wait'
                self.contact_started = now
                self._reset_pi()
            elif (self.marker_raw_visible and self.marker_visible and
                  self._marker_fresh(now)):
                self.reason = ''
                self._record_marker_growth(now)
                speed = self._visible_contact_speed()
                cmd = self._alignment_command(
                    now, speed, speed)
            elif (self.marker_visible and
                  self.marker_lost_since is not None and
                  now - self.marker_lost_since < float(
                      self._parameter('marker_disappear_confirm_s'))):
                # 원본 프레임에서 마커가 사라졌다. 검출이 한 프레임 튄
                # 것인지 진짜 사라진 것인지 가리려고 잠깐만 멈춘다 —
                # marker_disappear_confirm_s 까지만이다.
                #
                # 2026-09-01 실측: 예전에는 필터의 marker_visible 이 풀리기를
                # 무한정 기다렸는데, 그 해제 처리는 marker_stable 경로에만
                # 있어서 결합이 active 인 동안에는 영영 안 풀렸다. 그래서 이
                # 가지에 22초 갇혔다가 contact_visual_timeout 으로 빠졌고,
                # '가까워져서 마커가 사라진' 정상 상황인데도 블라인드 접촉으로
                # 넘어가지 못했다.
                self.reason = 'marker_loss_confirming'
                self._reset_pi()
            else:
                lost_for = (0.0 if self.marker_lost_since is None else
                            max(0.0, now - self.marker_lost_since))
                if lost_for >= float(
                        self._parameter('marker_disappear_confirm_s')):
                    # contact 단계에 들어왔다는 것 자체가 이미 정렬을
                    # 마치고 마커를 보며 접근 중이었다는 뜻이다. 그 상태에서
                    # 마커가 사라지는 것은 '가까워져서 화면 밖으로 나갔다'는
                    # 것이므로, 조건 없이 블라인드 직진으로 넘어간다.
                    #
                    # 예전에는 marker_growth_armed(0.8초 창의 크기 증가 추세)
                    # 를 요구했는데, 접근이 도중에 끊기거나 마커가 순식간에
                    # 프레임을 벗어나면 추세가 안 쌓여 영영 블라인드로 못
                    # 들어갔다. 그러면 _schedule_marker_retry 로 빠지고,
                    # 그 경로는 marker_wait 로 돌아가 다시 마커를 요구하는데
                    # 로봇이 이미 너무 가까워 마커가 영영 안 보이므로
                    # 빠져나올 수 없는 막다른 길이었다(2026-09-01 실측).
                    #
                    # 폭주 방지는 여기가 아니라 블라인드 자체의 한도가 한다:
                    # blind_contact_distance_m(30cm) 와 contact_timeout_s(4초).
                    self._enter_blind_contact(now)
                    cmd.linear.x = abs(float(
                        self._parameter('contact_speed_mps')))
                else:
                    # 원본 여러 프레임의 소실 확정이 끝날 때까지는 정지한다.
                    # 확정 뒤에도 증가 추세가 없으면 블라인드로 가지 않는다.
                    self.reason = 'marker_loss_confirming'
        elif self.active and self.phase == 'locking':
            self._publish_solenoid(True)
            if self.solenoid_locked:
                # 솔레노이드 상태만으로 성공 처리하지 않는다. 뒤로 당기면서
                # 하중 전류가 실제로 증가해야 최종 locked다. ArUco는 가까이서
                # 화면 밖으로 나간 뒤 블라인드 접촉 전환 시 꺼져 있다.
                self.phase = 'verify_pull'
                self.verify_started = now
                self.verify_current_hits = 0
                self.last_current_update = self.motor_update
            elif now - self.lock_started > float(self._parameter('solenoid_timeout_s')):
                self._abort('solenoid_timeout', unlock=True)
        elif self.active and self.phase == 'verify_pull':
            self._publish_solenoid(True)
            # 2026-09-03: 검증 실패에 솔레노이드를 풀지 않는다. 예전에는
            # 세 경로 모두 _abort(..., unlock=True) 라 '덜 붙었다'는
            # 신호에 잠금을 풀어 버렸다 — 솔레노이드는 기계적으로 잠기고
            # 해제는 사람이 누를 때만 한다는 확정 사항과 어긋난다.
            # 실패하면 결합을 다시 돌린다(TODO 21번).
            if not self.solenoid_locked:
                self._verify_retry('solenoid_unlocked')
            elif not self._current_ready():
                self._verify_retry('verify_current_lost')
            elif now - self.verify_started > float(
                    self._parameter('verify_timeout_s')):
                self._verify_retry('verify_timeout')
            else:
                # 설정 부호와 무관하게 검증 동작은 항상 후진이다.
                cmd.linear.x = -abs(float(
                    self._parameter('verify_pull_speed_mps')))
                if self.motor_update != self.last_current_update:
                    self.last_current_update = self.motor_update
                    threshold = self.verify_current_threshold()
                    self.verify_current_hits = (
                        self.verify_current_hits + 1
                        if self.current_a >= threshold else 0)
                current_loaded = self.verify_current_hits >= int(
                    self._parameter('verify_current_confirm_samples'))
                if current_loaded:
                    cmd = Twist()
                    self.active = False
                    self.phase = 'locked'
                    self.reason = ''
                    self.verify_retries = 0

        self.output = cmd
        self.cmd_pub.publish(cmd)

    def _publish_status(self):
        eligible, eligibility_reason = self._eligibility()
        stable_for = 0.0
        if self.marker_seen_since is not None:
            stable_for = max(0.0, time.monotonic() - self.marker_seen_since)
        marker_age = None
        if self.latest_marker_time is not None:
            marker_age = max(0.0, time.monotonic() - self.latest_marker_time)
        marker_status_age = None
        if self.marker_status_time is not None:
            marker_status_age = max(
                0.0, time.monotonic() - self.marker_status_time)
        status = String()
        status.data = json.dumps({
            'active': self.active,
            'start_pending': self.start_pending,
            'phase': self.phase,
            'eligible': eligible,
            'eligibility_reason': eligibility_reason,
            'reason': self.reason,
            # 사람에게 보이는 '인식' 상태는 현재 원본 프레임 기준이다.
            # 내부 다중 프레임 유지 상태는 별도 필드로 공개한다.
            'marker_visible': self.marker_raw_visible,
            'marker_filtered_visible': self.marker_visible,
            'marker_raw_visible': self.marker_raw_visible,
            'marker_stable': self.marker_stable,
            'marker_stable_for_s': stable_for,
            'marker_age_s': marker_age,
            'marker_status_age_s': marker_status_age,
            'camera_frame_fresh': self.marker_camera_fresh,
            'marker_detection_enabled': self.marker_detection_enabled,
            'marker_side_px': (
                self.latest_marker.get('side_px')
                if self.latest_marker is not None else None),
            'marker_blind_arm_side_px': self._parameter(
                'marker_blind_arm_side_px'),
            'contact_marker_near': self.contact_marker_near,
            'marker_growth_armed': self.marker_growth_armed,
            'marker_growth': self.marker_growth,
            'contact_visible_speed_mps': (
                self._visible_contact_speed()
                if self.phase == 'contact' and not self.contact_blind and
                self.marker_visible and self.latest_marker is not None
                else None),
            'contact_blind': self.contact_blind,
            'blind_contact_distance_m': self.blind_contact_distance_m,
            'blind_contact_distance_limit_m': self._parameter(
                'blind_contact_distance_m'),
            'reference': self.reference,
            'current_marker': self.latest_marker,
            'current_a': self.current_a,
            'current_threshold_a': self.contact_threshold(),
            'contact_current_hold_s': self._parameter(
                'contact_current_hold_s'),
            'contact_current_above_for_s': (
                0.0 if self.contact_current_since is None else
                max(0.0, time.monotonic() - self.contact_current_since)),
            'verify_current_threshold_a': self.verify_current_threshold(),
            'verify_current_hits': self.verify_current_hits,
            'shared_threshold_a': self.shared_threshold_a,
            'can_connected': self.motor_can,
            'solenoid_locked': self.solenoid_locked,
            'errors': self.errors,
            'pi': {
                'lateral_integral_ratio_s': self.lateral_integral,
                'lateral_rate': self.lateral_rate,
                'yaw_integral_deg_s': self.yaw_integral,
            },
            'linear': self.output.linear.x,
            'angular': self.output.angular.z,
        }, separators=(',', ':'))
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = Coupling()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
