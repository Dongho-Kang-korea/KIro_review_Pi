"""결합 보류, ArUco PI 추적/소실 handoff, 접촉·당김 전류 검증."""
import json
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from mission.common.coupling import Coupling


REFERENCE = {
    'x_mm': 0.0, 'y_mm': 0.0, 'z_mm': 300.0,
    'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0,
}
FAR_POSE = dict(REFERENCE, offset_x_px=0.0, side_px=150.0)
CLOSE_POSE = dict(
    REFERENCE, z_mm=240.0, offset_x_px=0.0, side_px=230.0)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Coupling()
    n.set_parameters([
        Parameter('contact_current_threshold_a', value=1.0),
        Parameter('contact_current_hold_s', value=0.5),
        Parameter('verify_current_confirm_samples', value=2),
        Parameter('marker_blind_arm_side_px', value=220.0),
        Parameter('marker_disappear_confirm_s', value=0.0),
        Parameter('blind_contact_distance_m', value=0.30),
        Parameter('initial_align_confirm_s', value=0.4),
        Parameter('verify_timeout_s', value=5.0),
    ])
    yield n
    n.destroy_node()


def _command(node, action):
    node._on_command(String(data=json.dumps({'action': action})))


def _make_start_ready(node):
    now = time.monotonic()
    node.reference = None
    node.marker_stable = False
    node.marker_visible = True
    node.marker_raw_visible = True
    node.latest_marker = dict(FAR_POSE)
    node.latest_marker_time = now
    node.marker_status_time = now
    node.motor_can = True
    node.current_a = 0.2
    node.current_time = now
    node.solenoid_locked = True


def _enter_verify_pull(node):
    now = time.monotonic()
    node.active = True
    node.phase = 'locking'
    node.lock_started = now
    node.solenoid_locked = True
    node.current_a = 0.2
    node.current_time = now
    node._tick()
    assert node.phase == 'verify_pull'
    assert node.active is True


def _current_sample(node, value):
    node.current_a = value
    node.current_time = time.monotonic()
    node.motor_update += 1
    node._tick()


def test_start_request_waits_and_retries_when_conditions_become_ready(node):
    _command(node, 'start')
    assert node.start_pending is True
    assert node.active is False
    assert node.reason == 'marker_status_wait'

    _make_start_ready(node)
    node._tick()

    assert node.start_pending is False
    assert node.active is True
    assert node.phase == 'align'
    assert node.reference == REFERENCE


def test_pending_start_waits_for_locked_solenoid_without_releasing(node):
    _make_start_ready(node)
    node.solenoid_locked = False
    published = []
    node.solenoid_pub.publish = published.append

    _command(node, 'start')
    assert node.start_pending is True
    assert node.reason == 'solenoid_unlocked'
    assert published == []

    node.solenoid_locked = True
    node._tick()
    assert node.start_pending is False
    assert node.active is True
    assert node.phase == 'align'
    assert published == []


def _gains(node):
    """조향 시험이 yaml/기본값 변화에 흔들리지 않게 게인을 고정한다."""
    node.set_parameters([
        Parameter('kp_bearing_rps_per_rad', value=0.6),
        Parameter('bearing_tolerance_deg', value=2.0),
        Parameter('max_angular_rps', value=0.05),
        Parameter('align_stable_samples', value=3),
    ])


def test_alignment_steers_by_bearing_and_only_on_new_samples(node):
    """방위각 P 제어 — 그리고 새 마커 표본이 왔을 때만 다시 계산한다.

    제어 tick(20Hz)이 마커(11Hz)보다 빨라서, 같은 표본에 두 번 반응하면
    적분이 배로 쌓이고 미분이 0 과 급변을 오간다(2026-09-01 실측).
    """
    _gains(node)
    node.reference = dict(REFERENCE)
    # atan2(20, 300) = 0.06656 rad = 3.813 deg -> -0.6 * 0.06656 = -0.0399
    node.latest_marker = dict(FAR_POSE, x_mm=20.0, offset_x_px=20.0)
    now = time.monotonic()
    cmd = node._alignment_command(now, 0.0)

    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(-0.0399, abs=1e-3)

    # 표본이 안 바뀌면 값이 달라져도 재계산하지 않고 직전 조향을 유지한다.
    node.latest_marker = dict(FAR_POSE, x_mm=-60.0, offset_x_px=-60.0)
    held = node._alignment_command(now + 0.05, 0.0)
    assert held.angular.z == pytest.approx(cmd.angular.z)

    # 새 표본이 오면 그때 반영한다(부호가 뒤집힌다).
    node.marker_seq += 1
    fresh = node._alignment_command(now + 0.1, 0.0)
    assert fresh.angular.z > 0.0


def test_alignment_stops_and_counts_inside_bearing_tolerance(node):
    """허용치 안에서는 조향을 0 으로 끊고 연속 표본을 센다.

    잔여 오차를 계속 쫓으면 관성과 겹쳐 넘어간다 — 2026-09-01 실측에서
    중심비 +-0.5 의 limit cycle 이 남았다.
    """
    _gains(node)
    node.reference = dict(REFERENCE)
    # atan2(8, 300) = 1.527 deg < 2.0
    node.latest_marker = dict(FAR_POSE, x_mm=8.0, offset_x_px=8.0)
    now = time.monotonic()

    for i in range(3):
        node.marker_seq += 1
        cmd = node._alignment_command(now + 0.1 * i, 0.0)
        assert cmd.angular.z == pytest.approx(0.0)
    assert node.aligned_samples == 3
    assert node._initial_aligned() is True

    # 한 번만 벗어나도 0 으로 리셋된다.
    node.latest_marker = dict(FAR_POSE, x_mm=40.0, offset_x_px=40.0)
    node.marker_seq += 1
    node._alignment_command(now + 0.5, 0.0)
    assert node.aligned_samples == 0
    assert node._initial_aligned() is False


def test_align_phase_corrects_bearing_without_approaching(node):
    _gains(node)
    now = time.monotonic()
    node.active = True
    node.phase = 'align'
    node.align_started = now
    node.reference = dict(REFERENCE)
    node.latest_marker = dict(FAR_POSE, x_mm=20.0, offset_x_px=20.0)
    node.latest_marker_time = now
    node.marker_status_time = now
    node.marker_visible = True
    node.marker_raw_visible = True
    node.current_a = 0.2
    node.current_time = now

    node._tick()

    assert node.phase == 'align'
    assert node.output.linear.x == pytest.approx(0.0)
    assert node.output.angular.z < 0.0


def test_initial_alignment_enters_contact_without_disabling_marker(node):
    _gains(node)
    now = time.monotonic()
    node.active = True
    node.phase = 'align'
    node.align_started = now
    node.aligned_since = now - 0.5
    node.aligned_samples = 5          # 이미 연속 표본을 채운 상태
    node.reference = dict(REFERENCE)
    node.latest_marker = dict(CLOSE_POSE)
    node.latest_marker_time = now
    node.marker_status_time = now
    node.marker_visible = True
    node.marker_raw_visible = True
    published = []
    node.marker_enable_pub.publish = published.append

    node._tick()

    assert node.phase == 'contact'
    assert node.marker_detection_enabled is True
    assert published == []
    assert node.output.linear.x > 0.0


def test_contact_tracks_visible_marker_by_bearing(node):
    _gains(node)
    now = time.monotonic()
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.reference = dict(REFERENCE)
    node.latest_marker = dict(CLOSE_POSE, x_mm=18.0, offset_x_px=18.0)
    node.latest_marker_time = now
    node.marker_status_time = now
    node.marker_visible = True
    node.marker_raw_visible = True
    node.current_a = 0.2
    node.current_time = now

    node._tick()

    assert node.phase == 'contact'
    assert node.contact_marker_near is True
    assert node.contact_blind is False
    assert node.marker_detection_enabled is True
    assert node.output.linear.x > 0.0
    assert node.output.angular.z < 0.0


def test_marker_loss_in_contact_always_enters_blind(node):
    """contact 에서 마커가 사라지면 조건 없이 블라인드 직진으로 넘어간다.

    contact 에 들어왔다는 것 자체가 정렬을 마치고 접근 중이었다는 뜻이라,
    거기서의 마커 소실은 '가까워져서 화면 밖으로 나갔다'로 본다.

    예전에는 marker_growth_armed(0.8초 창의 크기 증가 추세)를 요구했는데,
    추세가 안 쌓이면 _schedule_marker_retry 로 빠져 marker_wait 로 돌아갔다.
    그 상태는 다시 마커를 요구하지만 로봇이 이미 너무 가까워 마커가 영영
    안 보이므로 빠져나올 수 없었다(2026-09-01 실측).

    폭주 방지는 블라인드 자체의 한도(30cm / 4초)가 맡는다.
    """
    now = time.monotonic()
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.marker_growth_armed = False      # 추세가 없어도
    node.contact_marker_near = False      # 근접 플래그가 없어도
    node.marker_visible = False
    node.marker_lost_since = now - 0.1
    node.marker_status_time = now
    node.current_a = 0.2
    node.current_time = now

    node._tick()

    assert node.phase == 'contact'
    assert node.contact_blind is True
    assert node.output.linear.x > 0.0
    assert node.output.angular.z == pytest.approx(0.0)


def test_marker_growth_then_disappearance_switches_to_blind(node):
    now = time.monotonic()
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.reference = dict(REFERENCE)
    for index, side_px in enumerate((120, 130, 145, 160, 180, 200)):
        node.latest_marker = dict(CLOSE_POSE, side_px=side_px)
        node._record_marker_growth(now - 0.5 + index * 0.1)
    assert node.marker_growth_armed is True

    node.marker_visible = False
    node.marker_lost_since = now - 0.1
    node.marker_status_time = now
    node.current_a = 0.2
    node.current_time = now
    published = []
    node.marker_enable_pub.publish = published.append

    node._tick()

    assert node.phase == 'contact'
    assert node.contact_blind is True
    assert node.marker_detection_enabled is False
    assert published[-1].data is False
    assert node.output.linear.x == pytest.approx(0.10)
    assert node.output.angular.z == pytest.approx(0.0)


def test_visible_contact_speed_decreases_as_marker_grows(node):
    node.set_parameters([
        Parameter('marker_approach_speed_mps', value=0.10),
        Parameter('marker_near_speed_mps', value=0.04),
        Parameter('marker_blind_arm_side_px', value=220.0),
    ])
    node.contact_start_side_px = 100.0
    node.latest_marker = dict(CLOSE_POSE, side_px=100.0)
    far_speed = node._visible_contact_speed()
    node.latest_marker = dict(CLOSE_POSE, side_px=160.0)
    middle_speed = node._visible_contact_speed()
    node.latest_marker = dict(CLOSE_POSE, side_px=220.0)
    near_speed = node._visible_contact_speed()

    assert far_speed == pytest.approx(0.10)
    assert far_speed > middle_speed > near_speed
    assert near_speed == pytest.approx(0.04)


def test_visible_marker_starts_contact_hold_without_latching_immediately(node):
    """마커가 보이는 동안에도 전류 유지 타이머는 시작하되 바로 잠그지는 않는다.

    2026-09-01 이전에는 보이는 구간에서 전류를 아예 안 봤다. 마커가 접촉
    전에 사라진다는 가정이 기하상 성립하지 않아 그 가정을 없앴다. 대신
    contact_current_hold_s(0.5초) 유지 조건은 그대로라, 조향 중 순간적인
    전류 상승만으로는 locking 으로 넘어가지 않는다.
    """
    now = time.monotonic()
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.reference = dict(REFERENCE)
    node.latest_marker = dict(CLOSE_POSE)
    node.latest_marker_time = now
    node.marker_status_time = now
    node.marker_visible = True
    node.marker_raw_visible = True
    node.current_a = 2.0
    node.current_time = now
    node.motor_update += 1

    node._tick()

    assert node.phase == 'contact'
    assert node.contact_blind is False
    # 타이머는 시작하지만 아직 확정 전이다.
    assert node.contact_current_since is not None


def test_stale_marker_stream_holds_and_resumes_without_manual_restart(node):
    now = time.monotonic()
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.reference = dict(REFERENCE)
    node.latest_marker = dict(CLOSE_POSE)
    node.latest_marker_time = now
    node.marker_status_time = now - 1.0
    node.marker_visible = True
    node.marker_raw_visible = True
    node.current_a = 0.2
    node.current_time = now

    node._tick()

    assert node.active is True
    assert node.phase == 'contact'
    assert node.reason == 'marker_status_wait'
    assert node.output.linear.x == pytest.approx(0.0)

    resumed = time.monotonic()
    node.latest_marker_time = resumed
    node.marker_status_time = resumed
    node._tick()

    assert node.active is True
    assert node.phase == 'contact'
    assert node.reason == ''
    assert node.output.linear.x > 0.0


def _loss_scene(node, lost_ago):
    """contact 중 원본 프레임에서 마커가 사라진 상황을 만든다."""
    now = time.monotonic()
    node.set_parameters([
        Parameter('marker_disappear_confirm_s', value=0.5)])
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.reference = dict(REFERENCE)
    node.latest_marker = dict(CLOSE_POSE, x_mm=18.0, yaw_deg=2.0)
    node.latest_marker_time = now
    node.marker_status_time = now
    # 다중 프레임 필터는 아직 검출을 유지하지만 이 카메라 프레임에는 없다.
    node.marker_visible = True
    node.marker_raw_visible = False
    node.marker_growth_armed = False
    node.marker_lost_since = now - lost_ago
    node.current_a = 0.2
    node.current_time = now
    return now


def test_raw_marker_loss_waits_only_within_the_confirm_window(node):
    """소실 직후 잠깐은 멈춰서 한 프레임 튄 것인지 확인한다."""
    _loss_scene(node, lost_ago=0.2)      # 0.5초 창 안

    node._tick()

    assert node.active is True
    assert node.phase == 'contact'
    assert node.contact_blind is False
    assert node.reason == 'marker_loss_confirming'
    assert node.output.linear.x == pytest.approx(0.0)
    assert node.output.angular.z == pytest.approx(0.0)


def test_raw_marker_loss_enters_blind_after_the_confirm_window(node):
    """확인 시간이 지나면 블라인드 접촉으로 넘어간다.

    2026-09-01 실측: 예전에는 필터의 marker_visible 이 풀리기를 무한정
    기다렸는데, 그 해제는 marker_stable 경로에만 있어 결합이 active 인
    동안 영영 안 풀렸다. 이 가지에 22초 갇혔다가 contact_visual_timeout
    으로 빠져, 가까워져 마커가 사라진 정상 상황에서 블라인드로 못 갔다.
    """
    _loss_scene(node, lost_ago=0.6)      # 0.5초 창 밖

    node._tick()

    assert node.phase == 'contact'
    assert node.contact_blind is True
    assert node.output.linear.x > 0.0
    assert node.output.angular.z == pytest.approx(0.0)


def test_stale_camera_frames_do_not_count_as_marker_disappearance(node):
    now = time.monotonic()
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.marker_visible = False
    node.marker_lost_since = now - 1.0
    node.marker_status_time = now
    node.marker_camera_fresh = False
    node.current_a = 0.2
    node.current_time = now

    node._tick()

    assert node.active is True
    assert node.phase == 'contact'
    assert node.contact_blind is False
    assert node.reason == 'camera_frame_wait'
    assert node.output.linear.x == pytest.approx(0.0)


def test_blind_distance_limit_stops_and_queues_automatic_retry(node):
    now = time.monotonic()
    node.set_parameters([
        Parameter('blind_contact_distance_m', value=0.01),
        Parameter('contact_speed_mps', value=0.10),
    ])
    node.active = True
    node.phase = 'contact'
    node.contact_blind = True
    node.blind_contact_started = now
    node.blind_last_tick = now - 0.1
    node.blind_contact_distance_m = 0.009
    node.marker_detection_enabled = False
    node.current_a = 0.2
    node.current_time = now

    node._tick()

    assert node.active is False
    assert node.start_pending is True
    assert node.phase == 'marker_wait'
    assert node.reason == 'blind_distance_limit_retry'
    assert node.marker_detection_enabled is True
    assert node.output.linear.x == pytest.approx(0.0)


def test_contact_current_must_stay_high_for_configured_time(node):
    now = time.monotonic()
    node.active = True
    node.phase = 'contact'
    node.contact_started = now
    node.contact_blind = True
    node.blind_contact_started = now
    node.current_a = 0.2
    node.current_time = now

    _current_sample(node, 2.0)
    assert node.phase == 'contact'
    assert node.contact_current_since is not None

    _current_sample(node, 0.2)
    assert node.phase == 'contact'
    assert node.contact_current_since is None

    _current_sample(node, 2.0)
    node.contact_current_since -= 0.51
    _current_sample(node, 2.0)

    assert node.phase == 'locking'
    assert node.output.linear.x == pytest.approx(0.0)


def test_solenoid_lock_alone_does_not_complete_coupling(node):
    _enter_verify_pull(node)
    assert node.phase != 'locked'


def test_low_pull_current_does_not_confirm_locked(node):
    _enter_verify_pull(node)
    _current_sample(node, 0.2)
    _current_sample(node, 0.2)

    assert node.verify_current_hits == 0
    assert node.phase == 'verify_pull'
    assert node.output.linear.x < 0.0


def test_pull_current_alone_confirms_locked_without_marker(node):
    _enter_verify_pull(node)
    node.marker_detection_enabled = False
    node.latest_marker = None

    _current_sample(node, 2.0)
    assert node.phase == 'verify_pull'
    assert node.output.linear.x < 0.0
    _current_sample(node, 2.0)

    assert node.active is False
    assert node.phase == 'locked'
    assert node.output.linear.x == pytest.approx(0.0)
    assert node.marker_detection_enabled is False


def test_contact_waits_safely_for_current(node):
    node.active = True
    node.phase = 'contact'
    node.contact_started = time.monotonic()
    node.current_a = None
    node.current_time = None
    node.marker_detection_enabled = False
    published = []
    node.marker_enable_pub.publish = published.append

    node._tick()

    assert node.active is True
    assert node.phase == 'contact'
    assert node.reason == 'current_unavailable'
    assert node.output.linear.x == pytest.approx(0.0)
    assert node.output.angular.z == pytest.approx(0.0)
    assert node.marker_detection_enabled is False
    assert published == []


def test_verify_timeout_retries_coupling_without_unlocking(node):
    """검증 실패는 '덜 붙었다'는 뜻 — 풀지 말고 다시 결합한다.

    2026-09-03 확정(TODO 21번). 예전에는 _abort(..., unlock=True) 로
    빠져 솔레노이드를 풀어 버렸다. 솔레노이드는 기계적으로 잠기고
    해제는 사람이 누를 때만 한다.
    """
    _enter_verify_pull(node)
    node.marker_detection_enabled = False
    node.verify_started = time.monotonic() - 6.0
    solenoid = []
    marker = []
    node.solenoid_pub.publish = solenoid.append
    node.marker_enable_pub.publish = marker.append

    node._tick()

    assert node.phase == 'marker_wait'      # error 가 아니라 재시도
    assert node.reason == 'verify_timeout'
    assert node.start_pending is True
    assert node.verify_retries == 1
    # 해제 신호(False)는 한 번도 나가지 않는다.
    assert all(msg.data is not False for msg in solenoid)
    assert marker[-1].data is True


def test_verify_retry_limit_stops_but_still_never_unlocks(node):
    """반복해도 안 붙으면 멈춘다 — 그때도 솔레노이드는 안 푼다."""
    _enter_verify_pull(node)
    node.verify_retries = int(node._parameter('verify_retry_limit'))
    node.verify_started = time.monotonic() - 6.0
    solenoid = []
    node.solenoid_pub.publish = solenoid.append

    node._tick()

    assert node.phase == 'error'
    assert node.reason == 'verify_timeout'
    assert all(msg.data is not False for msg in solenoid)


def test_contact_current_is_checked_while_the_marker_is_still_visible(node):
    """마커가 안 사라져도 접촉 전류로 locking 에 들어간다.

    fx 1453 px / 마커 40 mm 기하에서 마커가 화면(720 px)을 채우는 거리는
    약 81 mm 라, 결합봉이 먼저 닿아 마커가 끝까지 보인 채 접촉한다. 예전에는
    이 판정이 블라인드 구간에만 있어 그 경우를 놓쳤다.
    """
    node.reference = dict(REFERENCE)
    node.latest_marker = dict(CLOSE_POSE)
    node.latest_marker_time = time.monotonic()
    node.marker_status_time = node.latest_marker_time
    node.marker_visible = True
    node.marker_raw_visible = True
    node.active = True
    node.phase = 'contact'
    node.contact_blind = False          # 아직 마커가 보인다
    node.contact_started = time.monotonic()
    node.current_a = 5.0                # 접촉 기준(1.0) 을 크게 넘김
    node.current_time = time.monotonic()
    node.last_current_update = -1

    # 기준 이상이 contact_current_hold_s 동안 유지되어야 확정된다.
    node.motor_update = 1
    node._tick()
    assert node.phase == 'contact'      # 아직 유지 시간 미달

    node.contact_current_since = time.monotonic() - 1.0
    node.motor_update = 2
    node._tick()

    assert node.phase == 'locking'
    assert node.output.linear.x == pytest.approx(0.0)

def test_cancel_stops_without_unlocking_and_unlock_is_the_only_release(node):
    """취소와 해제를 분리했다(2026-09-03, TODO 23번).

    예전에는 cancel 과 unlock 이 같은 분기라, 결합을 취소할 때마다
    잠금까지 풀렸다.
    """
    node.active = True
    node.phase = 'contact'
    solenoid = []
    node.solenoid_pub.publish = solenoid.append

    node._on_command(String(data=json.dumps({'action': 'cancel'})))
    assert node.active is False
    assert node.phase == 'ready'
    assert all(msg.data is not False for msg in solenoid)

    node._on_command(String(data=json.dumps({'action': 'unlock'})))
    assert solenoid[-1].data is False


def test_fleet_couple_action_starts_coupling_directly(node):
    """1호기의 couple 을 coupling 이 직접 받는다(TODO 24번).

    예전에는 여름 노드 cargo_load 가 받아서 coupling/cmd 로 넘겨줬다.
    """
    node.start_pending = False
    node._on_fleet_action(String(data=json.dumps({'action': 'couple'})))
    assert node.start_pending is True

    # 다른 액션은 무시한다 — 화물은 cargo_load 몫이다.
    node.start_pending = False
    node._on_fleet_action(String(data=json.dumps({'action': 'load'})))
    assert node.start_pending is False
