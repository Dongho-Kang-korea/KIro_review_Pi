"""coupling.py가 새 ArUco pose 필드명(x_mm/y_mm/z_mm/roll_deg/pitch_deg/
yaw_deg)을 제대로 읽는지 확인한다 — 예전 4색 마커 필드명(span_px/lateral/
roll/pitch/yaw)을 계속 찾던 버그의 회귀 테스트다(2026-08-28 재배선).
"""
import math

import pytest
import rclpy

from mission.common.coupling import Coupling, angle_error


def _status(**overrides):
    """marker_vision._publish_status()가 내보내는 marker/status 모양."""
    base = {
        'detected': True,
        'x_mm': 12.0, 'y_mm': -3.0, 'z_mm': 300.0,
        'roll_deg': 1.5, 'pitch_deg': 2.5, 'yaw_deg': -4.0,
        'offset_x_px': 14.0, 'side_px': 140.0,
    }
    base.update(overrides)
    return base


def test_marker_values_accepts_new_aruco_pose_fields():
    result = Coupling._marker_values(_status())
    assert result == {
        'x_mm': 12.0, 'y_mm': -3.0, 'z_mm': 300.0,
        'roll_deg': 1.5, 'pitch_deg': 2.5, 'yaw_deg': -4.0,
        'offset_x_px': 14.0, 'side_px': 140.0,
    }


def test_marker_values_rejects_old_four_color_fields_only():
    # 예전 필드명만 있고(x_mm 없음) detected인 경우 — 재배선 전에는 이걸
    # 계속 값으로 오인했다. 지금은 None이어야 한다.
    old_style = {
        'detected': True,
        'span_px': 120.0, 'lateral': 0.01,
        'roll': 1.0, 'pitch': 2.0, 'yaw': 3.0,
    }
    assert Coupling._marker_values(old_style) is None


def test_marker_values_rejects_not_detected():
    assert Coupling._marker_values(_status(detected=False)) is None


def test_marker_values_rejects_pose_unavailable():
    # 마커는 검출됐지만(2D만) pose_available=False라 x_mm 등이 안 실린 경우.
    value = {'detected': True, 'target_detected': True}
    assert Coupling._marker_values(value) is None


def test_marker_values_rejects_non_finite():
    assert Coupling._marker_values(_status(z_mm=float('nan'))) is None
    assert Coupling._marker_values(_status(x_mm=float('inf'))) is None


def test_angle_error_wraps_to_shortest_path():
    assert angle_error(179.0, -179.0) == pytest.approx(-2.0)
    assert angle_error(-179.0, 179.0) == pytest.approx(2.0)
    assert angle_error(10.0, 5.0) == pytest.approx(5.0)


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Coupling()
    yield n
    n.destroy_node()


def test_on_marker_applies_z_ema_smoothing(node):
    import json
    alpha = 0.35
    node.set_parameters([
        rclpy.parameter.Parameter('z_ema_alpha', value=alpha)])

    class Msg:
        pass

    m1 = Msg(); m1.data = json.dumps(_status(z_mm=300.0))
    node._on_marker(m1)
    assert node.latest_marker['z_mm'] == pytest.approx(300.0)

    m2 = Msg(); m2.data = json.dumps(_status(z_mm=340.0))
    node._on_marker(m2)
    expected = (1.0 - alpha) * 300.0 + alpha * 340.0
    assert node.latest_marker['z_mm'] == pytest.approx(expected)
    # 스무딩은 z만 걸린다 — x/y/roll/pitch/yaw는 원값 그대로.
    assert node.latest_marker['x_mm'] == pytest.approx(12.0)


def test_reference_acquisition_survives_brief_detection_dropout(node):
    import json
    import time

    class Msg:
        pass

    detected = Msg(); detected.data = json.dumps(_status())
    missing = Msg(); missing.data = json.dumps({'detected': False})

    node._on_marker(detected)
    first_seen = node.marker_seen_since
    assert first_seen is not None
    assert len(node.marker_samples) == 1

    node._on_marker(missing)
    assert node.marker_seen_since == first_seen
    assert len(node.marker_samples) == 1

    node.marker_lost_since = time.monotonic() - 0.4
    node._on_marker(missing)
    assert node.marker_seen_since is None
    assert len(node.marker_samples) == 0


def test_alignment_errors_expose_pose_fields_and_bearing(node):
    """pose 필드 배선 회귀 + 방위각 계산 확인.

    완료 판정은 2026-09-01 부터 방위각의 **연속 표본 수**로 바뀌었다
    (아래 test_coupling_verification.py 참고). 여기서는 오차 계산만 본다.
    """
    node.reference = {
        'x_mm': 0.0, 'y_mm': 0.0, 'z_mm': 300.0,
        'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 0.0,
    }
    node.latest_marker = {
        'x_mm': 50.0, 'y_mm': 0.0, 'z_mm': 360.0,
        'roll_deg': 1.0, 'pitch_deg': 0.0, 'yaw_deg': 3.0,
        'offset_x_px': 18.0, 'side_px': 180.0,
    }
    errors = node._update_alignment_errors()
    assert errors['distance_m'] == pytest.approx(0.06)
    assert errors['lateral_m'] == pytest.approx(0.05)
    assert errors['lateral_marker_ratio'] == pytest.approx(0.10)
    assert errors['yaw_deg'] == pytest.approx(3.0)
    assert errors['marker_side_px'] == pytest.approx(180.0)
    # atan2(50, 360) = 0.13803 rad = 7.907 deg
    assert errors['bearing_deg'] == pytest.approx(7.907, abs=1e-2)

    # 표본을 안 쌓았으면 아직 정렬 완료가 아니다.
    assert node._initial_aligned() is False


def test_capture_reference_uses_new_field_names(node):
    import time
    now = time.monotonic()
    samples = [
        {'x_mm': 10.0, 'y_mm': 1.0, 'z_mm': 300.0,
         'roll_deg': 1.0, 'pitch_deg': 1.0, 'yaw_deg': 1.0},
        {'x_mm': 12.0, 'y_mm': 1.0, 'z_mm': 302.0,
         'roll_deg': 1.0, 'pitch_deg': 1.0, 'yaw_deg': 1.0},
        {'x_mm': 14.0, 'y_mm': 1.0, 'z_mm': 304.0,
         'roll_deg': 1.0, 'pitch_deg': 1.0, 'yaw_deg': 1.0},
    ]
    node.marker_samples.extend((now, s) for s in samples)
    node._capture_reference(now)
    assert node.reference == {
        'x_mm': 12.0, 'y_mm': 1.0, 'z_mm': 302.0,
        'roll_deg': 1.0, 'pitch_deg': 1.0, 'yaw_deg': 1.0,
    }
    assert node.phase == 'ready'
