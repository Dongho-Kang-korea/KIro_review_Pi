"""follow_leader.py의 marker/pose(PoseStamped) 처리 — pose가 다시 나오기
시작한 뒤(2026-08-28 ArUco 캘리브레이션 확정), z(거리) EMA 스무딩이 UWB
폴백에서 실제로 쓰이는지 확인한다. x/z 필드 자체는 원래도 PoseStamped
타입 구독이라 필드명 문제가 없었다 — 여기서는 그 사실과 새로 추가한
스무딩만 검증한다.

2026-08-29: yaw(정면 정렬) 비례항 추가에 맞춰 두 가지를 더 검증한다 —
(1) 쿼터니언 -> heading 역변환이 marker_vision.py의 순변환과 정확히 짝을
이루는지(왕복 검증), (2) 좌우 오프셋항 + yaw항이 실제로 더해지는지와
k_yaw 기본값(0.0)에서는 기존 동작과 완전히 같은지(회귀 없음).

2026-08-30: 실측으로 축 오인식 발견 — 로봇의 실제 좌우 회전(월드 yaw)은
aruco_tracker.py 기준 `yaw_deg`(마커 로컬 Z축, 이미지 평면 회전)가 아니라
`pitch_deg`(마커 로컬 Y축 회전)로 나타난다(이 로봇은 마커판이 수직으로
붙어있어서). `_yaw_from_quaternion`을 `_heading_from_quaternion`으로
바꾸고 쿼터니언의 pitch(Y축) 성분을 뽑도록 수정 — 아래 왕복 검증 테스트도
이에 맞춰 pitch_deg를 복원하는지로 바꿨다.
"""
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter

from mission.winter.follow_leader import FollowLeader


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = FollowLeader()
    yield n
    n.destroy_node()


def _pose(x_m, z_m):
    msg = PoseStamped()
    msg.pose.position.x = x_m
    msg.pose.position.z = z_m
    return msg


def test_on_pose_smooths_z_but_not_x(node):
    alpha = 0.3
    node.set_parameters([
        rclpy.parameter.Parameter('marker_distance_ema_alpha', value=alpha)])

    node._on_pose(_pose(x_m=0.02, z_m=0.30))
    assert node.marker_distance_ema_mm == pytest.approx(300.0)

    node._on_pose(_pose(x_m=0.05, z_m=0.36))
    expected = (1.0 - alpha) * 300.0 + alpha * 360.0
    assert node.marker_distance_ema_mm == pytest.approx(expected)
    # 원본 pose는 그대로 저장돼 lateral(x) 계산에는 스무딩이 안 걸린다.
    assert node.pose.pose.position.x == pytest.approx(0.05)


def test_marker_timeout_resets_distance_ema(node):
    node.action = 'follow'
    node._on_pose(_pose(x_m=0.0, z_m=0.30))
    assert node.marker_distance_ema_mm is not None
    # pose_time을 과거로 밀어 marker_timeout을 강제한다.
    node.pose_time = time.monotonic() - 10.0
    node._tick()
    assert node.marker_distance_ema_mm is None


def _quaternion_from_rpy(roll, pitch, yaw):
    """marker_vision.py `_publish_pose`와 정확히 같은 순변환(테스트 전용
    재구현) — `_heading_from_quaternion`이 그 역변환과 정확히 짝이 맞는지
    검증하는 기준값을 만든다."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,   # w
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
    )


def test_heading_from_quaternion_roundtrips_pitch_axis():
    """`_heading_from_quaternion`은 쿼터니언의 yaw(Z축) 성분이 아니라
    pitch(Y축) 성분을 복원해야 한다 — 이 로봇의 마커 마운트 기준으로
    pitch_deg가 실제 로봇의 좌우 회전(월드 yaw)이기 때문이다(2026-08-30
    실측으로 확인, 위 모듈 docstring 참고)."""
    for roll_deg, pitch_deg, yaw_deg in (
            (0.0, 0.0, 0.0), (3.0, -7.0, 20.0), (-10.0, 15.0, -60.0),
            (5.0, 5.0, 179.0), (0.0, 0.0, -175.0)):
        roll, pitch, yaw = (math.radians(v) for v in
                            (roll_deg, pitch_deg, yaw_deg))
        w, x, y, z = _quaternion_from_rpy(roll, pitch, yaw)
        orientation = PoseStamped().pose.orientation
        orientation.w, orientation.x, orientation.y, orientation.z = w, x, y, z
        recovered = FollowLeader._heading_from_quaternion(orientation)
        assert recovered == pytest.approx(pitch, abs=1e-9), (
            f'roll={roll_deg} pitch={pitch_deg} yaw={yaw_deg}')


def _pose_with_yaw(x_m, z_m, yaw_rad):
    """`yaw_rad`는 로봇의 실제 좌우 회전(월드 yaw) — 마커 자신의 로컬
    Y축(pitch) 회전으로 인코드한다(`_heading_from_quaternion` 참고,
    순수 Y축 회전이라 쿼터니언은 w=cos(θ/2), y=sin(θ/2)만 0이 아니다)."""
    msg = PoseStamped()
    msg.pose.position.x = x_m
    msg.pose.position.z = z_m
    msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
    msg.pose.orientation.y = math.sin(yaw_rad / 2.0)
    return msg


def test_combine_angular_adds_lateral_and_yaw_terms(node):
    node.set_parameters([
        Parameter('k_angular', value=2.0),
        Parameter('k_yaw', value=0.5),
        Parameter('lateral_tolerance_m', value=0.0),
        Parameter('yaw_tolerance_deg', value=0.0),
        Parameter('max_angular_rps', value=10.0),  # 클램프에 안 걸리게 크게
    ])
    yaw = math.radians(20.0)
    result = node._combine_angular(lateral=0.1, yaw_rad=yaw)
    assert result == pytest.approx(-2.0 * 0.1 + 0.5 * yaw)


def test_combine_angular_clamps_to_max_angular_rps(node):
    node.set_parameters([
        Parameter('k_angular', value=100.0),
        Parameter('k_yaw', value=0.0),
        Parameter('lateral_tolerance_m', value=0.0),
        Parameter('max_angular_rps', value=0.5),
    ])
    assert node._combine_angular(lateral=1.0, yaw_rad=0.0) == pytest.approx(-0.5)
    assert node._combine_angular(lateral=-1.0, yaw_rad=0.0) == pytest.approx(0.5)


def test_default_k_yaw_is_zero_no_behaviour_change(node):
    """k_yaw 기본값(0.0)에서는 yaw가 아무리 커도 조향에 영향이 없어야
    한다 — 기존(yaw 항 도입 전) 동작과 완전히 동일해야 회귀가 없다."""
    node.set_parameters([Parameter('lateral_tolerance_m', value=0.0)])
    only_lateral = node._combine_angular(lateral=0.05, yaw_rad=0.0)
    with_large_yaw = node._combine_angular(lateral=0.05, yaw_rad=math.radians(90.0))
    assert with_large_yaw == pytest.approx(only_lateral)


def test_tick_sets_angular_from_pose_with_yaw(node):
    node.action = 'follow'
    node.set_parameters([
        Parameter('allow_marker_distance_fallback', value=True),
        Parameter('k_angular', value=0.0),
        Parameter('k_yaw', value=0.5),
        Parameter('yaw_tolerance_deg', value=0.0),
        Parameter('max_angular_rps', value=10.0),
        Parameter('distance_tolerance_mm', value=1000.0),  # linear 항은 0으로 고정
    ])
    yaw = math.radians(-30.0)
    node._on_pose(_pose_with_yaw(x_m=0.0, z_m=0.60, yaw_rad=yaw))
    node._tick()
    assert node.yaw_rad == pytest.approx(yaw)
