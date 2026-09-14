"""Multi-frame confidence and hysteresis for ArUco detections."""
import json
import time

import pytest
import rclpy

from base.marker_vision import MarkerVision


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    value = MarkerVision()
    yield value
    value.destroy_node()


def _good_result():
    return {
        'detected': True,
        'target_detected': True,
        'detected_ids': [1],
        'decoded_ids': [1],
        'rejected_ids': [],
        'markers': [],
        'x_mm': 0.0,
        'y_mm': 0.0,
        'z_mm': 300.0,
        'roll_deg': 0.0,
        'pitch_deg': 0.0,
        'yaw_deg': 0.0,
        'side_px': 100.0,
    }


def _feed(node, start, pattern, period=1.0 / 30.0):
    states = []
    for index, detected in enumerate(pattern):
        now = start + index * period
        if detected:
            node.last_detection_time = now
            node.last_detection_result = _good_result()
        states.append(node._update_detection_state(now, detected))
    return states


def test_acquires_from_multiple_hits_not_one_frame(node):
    states = _feed(node, 10.0, [True, False, True, True, True])

    assert states[0] is False
    assert states[-1] is True
    assert node.window_hits == 4
    assert node.window_hit_ratio == pytest.approx(0.8)


def test_intermittent_misses_do_not_drop_tracked_marker(node):
    _feed(node, 20.0, [True, True, True, True])
    states = _feed(
        node, 20.2,
        [False, True, False, True, False, True, False, True])

    assert all(states)
    assert node.filtered_detected is True


def test_sustained_multi_frame_loss_releases_after_quarter_second(node):
    _feed(node, 30.0, [True, True, True, True])
    states = _feed(node, 30.2, [False] * 10)

    assert states[0] is True
    assert states[-1] is False
    assert node.window_samples >= 6
    assert 1.0 - node.window_hit_ratio >= 0.8


def test_filtered_status_keeps_last_pose_across_raw_miss(node):
    now = time.monotonic()
    node.last_result = node._empty_result()
    node.last_detection_result = _good_result()
    node.last_frame_time = now
    node.last_detection_time = now - 0.05
    node.filtered_detected = True
    node.window_samples = 9
    node.window_hits = 7
    node.window_hit_ratio = 7.0 / 9.0
    published = []
    node.status_pub.publish = published.append

    node._publish_status()
    status = json.loads(published[-1].data)

    assert status['raw_detected'] is False
    assert status['detected'] is True
    assert status['x_mm'] == pytest.approx(0.0)
    assert status['detection_window_samples'] == 9
    assert status['detection_window_hits'] == 7
