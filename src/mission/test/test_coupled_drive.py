"""결합 협조 주행의 ROS와 무관한 속도 변환 계산."""
import pytest

from mission.common.coupled_drive import clamp, coupled_command


def command(linear, angular=None, *, linear_sign=1.0, angular_sign=1.0,
            follow_angular=False, max_linear=0.35, max_angular=0.30):
    return coupled_command(linear, angular, linear_sign, angular_sign,
                           follow_angular, max_linear, max_angular)


def test_leader_speed_is_copied_when_sign_is_positive():
    assert command(0.20) == (pytest.approx(0.20), 0.0)


def test_negative_sign_flips_direction_for_reversed_coupling_geometry():
    assert command(0.20, linear_sign=-1.0) == (pytest.approx(-0.20), 0.0)


def test_linear_speed_is_clamped_to_the_configured_limit():
    assert command(9.0)[0] == pytest.approx(0.35)
    assert command(-9.0)[0] == pytest.approx(-0.35)


def test_angular_is_suppressed_when_disabled_for_a_hinged_coupling():
    # 강체 결합에서는 켠다(coupled.yaml 참고). 이 경로는 힌지형으로 바꿔
    # 껐을 때를 위한 것이다 — 조인트 각을 모르면 조향을 복제하면 안 된다.
    assert command(0.20, 0.9) == (pytest.approx(0.20), 0.0)


def test_angular_passes_through_and_clamps_when_explicitly_enabled():
    linear, angular = command(0.20, 0.9, follow_angular=True)
    assert linear == pytest.approx(0.20)
    assert angular == pytest.approx(0.30)


def test_angular_sign_is_independent_of_linear_sign():
    _, angular = command(0.20, 0.1, linear_sign=-1.0, angular_sign=-1.0,
                         follow_angular=True)
    assert angular == pytest.approx(-0.1)


def test_missing_leader_speed_stops_instead_of_guessing():
    assert command(None) == (0.0, 0.0)


def test_missing_leader_angular_still_allows_straight_driving():
    assert command(0.20, None, follow_angular=True) == (pytest.approx(0.20), 0.0)


def test_clamp_is_symmetric_regardless_of_limit_sign():
    assert clamp(5.0, -2.0) == pytest.approx(2.0)
    assert clamp(-5.0, 2.0) == pytest.approx(-2.0)
