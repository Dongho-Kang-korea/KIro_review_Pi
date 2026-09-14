"""Pure calculations used by the live IMU/ArUco 360 degree test."""
import pytest

from base.imu_aruco_360_test import (
    HeadingAccumulator,
    alignment_angular_rps,
    angle_error_deg,
    circular_mean_deg,
)


def test_heading_accumulator_detects_one_ccw_turn_across_wrap():
    accumulator = HeadingAccumulator(350.0)
    for heading in (
            355.0, 0.0, 45.0, 90.0, 135.0, 180.0,
            225.0, 270.0, 315.0, 350.0):
        accumulator.update(heading)
    assert accumulator.total_deg == pytest.approx(360.0)


def test_heading_accumulator_reports_clockwise_as_negative():
    accumulator = HeadingAccumulator(10.0)
    accumulator.update(0.0)
    accumulator.update(350.0)
    assert accumulator.total_deg == pytest.approx(-20.0)


def test_heading_accumulator_rejects_sensor_jump():
    accumulator = HeadingAccumulator(0.0, max_step_deg=45.0)
    with pytest.raises(ValueError, match='IMU heading jump'):
        accumulator.update(100.0)


def test_angle_error_uses_shortest_signed_difference():
    assert angle_error_deg(2.0, 358.0) == pytest.approx(4.0)
    assert angle_error_deg(358.0, 2.0) == pytest.approx(-4.0)


def test_circular_mean_handles_wrap_boundary():
    mean = circular_mean_deg([179.0, -179.0, 180.0])
    assert abs(abs(mean) - 180.0) < 0.1


def test_alignment_command_reuses_coupling_steering_sign_and_limits():
    assert alignment_angular_rps(10.0, 0.01, 0.05, 0.18) == pytest.approx(-0.1)
    assert alignment_angular_rps(-1.0, 0.01, 0.05, 0.18) == pytest.approx(0.05)
    assert alignment_angular_rps(100.0, 0.01, 0.05, 0.18) == pytest.approx(-0.18)
