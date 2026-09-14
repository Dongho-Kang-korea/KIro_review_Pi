"""새 적재 순서: 1호기 판정 수신, 2호기 자체 전류 하락 완료."""
import json
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String

from mission.summer.cargo_load import CargoLoad


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = CargoLoad()
    n.set_parameters([
        Parameter('contact_uwb_threshold_mm', value=900.0),
        Parameter('contact_confirm_samples', value=3),
        Parameter('unit2_load_high_current_a', value=1.0),
        Parameter('unit2_load_low_current_a', value=0.5),
        Parameter('load_min_duration_s', value=0.0),
        Parameter('load_complete_hold_s', value=0.1),
        Parameter('loading_timeout_s', value=5.0),
    ])
    n.published_drive = []
    n.published_roller = []
    n.cmd_pub.publish = n.published_drive.append
    n.roller_pub.publish = n.published_roller.append
    yield n
    n.destroy_node()


def _action(node, action, **extra):
    node._on_action(String(data=json.dumps({'action': action, **extra})))


def _local_current(node, current, seq=1, roller_current=9.0):
    node._on_motor(String(data=json.dumps({
        'current_seq': seq,
        'drive_motor_ids': [1, 2],
        'motors': {
            '1': {'current_a': current, 'current_fresh': True},
            '2': {'current_a': current * 0.9, 'current_fresh': True},
            # 롤러 전류가 커도 적재 완료용 주행 전류에는 섞이면 안 된다.
            '9': {'current_a': roller_current, 'current_fresh': True},
        },
    })))


def _leader_status(node, current=0.0, *, contact=False, dropped=False,
                   detection_ready=True):
    node._on_leader_status(String(data=json.dumps({
        'unit': 1, 'drive_current_a': current,
        'cargo_contact_detected': contact,
        'cargo_load_current_dropped': dropped,
        'cargo_current_detection_ready': detection_ready,
    })))


def _uwb(node, distance_mm):
    node._on_uwb(Float32(data=distance_mm))


def test_approach_immediately_turns_then_waits_stationary(node):
    node._on_heading(Float32(data=10.0))
    _action(node, 'approach')
    assert node.phase == 'turning'
    assert node.turn_target_deg == pytest.approx(180.0)

    node._tick()
    assert node.published_drive[-1].angular.z > 0.0
    assert node.published_drive[-1].linear.x == pytest.approx(0.0)

    node._on_heading(Float32(data=185.0))
    node._tick()
    assert node.phase == 'approach'
    assert node.published_drive[-1].linear.x == pytest.approx(0.0)
    assert node.published_drive[-1].angular.z == pytest.approx(0.0)


def test_unit2_does_not_back_toward_cargo_while_waiting(node):
    node.action = 'approach'
    node.phase = 'approach'
    _uwb(node, 1200.0)
    _local_current(node, 5.0)
    node._tick()
    assert node.phase == 'approach'
    assert node.published_drive[-1].linear.x == pytest.approx(0.0)


def test_contact_requires_uwb_gate_and_three_new_leader_samples(node):
    node.action = 'approach'
    node.phase = 'approach'
    _uwb(node, 850.0)
    _local_current(node, 0.1)

    for expected_hits in (1, 2, 3):
        _leader_status(node, 1.2, contact=True)
        node._tick()
        assert node.contact_hits == expected_hits
    assert node.phase == 'contact'


def test_high_leader_current_is_ignored_when_uwb_is_far(node):
    node.action = 'approach'
    node.phase = 'approach'
    _uwb(node, 901.0)
    for _ in range(4):
        _leader_status(node, 2.0, contact=True)
        node._tick()
    assert node.phase == 'approach'
    assert node.contact_hits == 0


def test_early_load_is_pending_and_roller_stays_off_until_contact(node):
    node.action = 'approach'
    node.phase = 'approach'
    _action(node, 'load', roller=0.8)
    node._tick()
    assert node.load_requested is True
    assert node.phase == 'approach'
    assert node.published_roller[-1].data == pytest.approx(0.0)

    _uwb(node, 850.0)
    for _ in range(3):
        _leader_status(node, 1.2, contact=True)
        node._tick()
    assert node.phase == 'loading'


def test_loading_requires_high_then_accepts_either_unit_drop(node):
    node.action = 'load'
    node.phase = 'contact'
    _local_current(node, 1.2, seq=1)
    _leader_status(node, 1.2)
    _action(node, 'load', roller=0.8)
    node._tick()
    assert node.phase == 'loading'
    assert node.published_drive[-1].linear.x < 0.0
    assert node.published_roller[-1].data == pytest.approx(0.8)

    # 2호기 자체 상승 전류가 먼저 기록된다.
    assert node.load_high_seen is True

    # 2호기 주행 전류만 낮아져도 유지시간 뒤 완료된다. 롤러 전류는 무시한다.
    _local_current(node, 0.3, seq=2)
    _leader_status(node, 1.2)
    node._tick()
    assert node.phase == 'loading'
    assert node.load_drop_since is not None
    node.load_drop_since = time.monotonic() - 0.2
    _local_current(node, 0.3, seq=3)
    _leader_status(node, 1.2)
    node._tick()
    assert node.phase == 'loaded'
    assert node.published_drive[-1].linear.x == pytest.approx(0.0)
    assert node.published_roller[-1].data == pytest.approx(0.0)


def test_loading_does_not_accept_low_current_before_high(node):
    node.action = 'load'
    node.phase = 'contact'
    _local_current(node, 0.3, seq=10)
    _leader_status(node, 0.4)
    _action(node, 'load', roller=0.8)
    for seq in range(11, 15):
        _local_current(node, 0.3, seq=seq)
        _leader_status(node, 0.4)
        node._tick()
    assert node.phase == 'loading'
    assert node.load_high_seen is False
    assert node.load_drop_since is None


def test_loading_refuses_unmeasured_current_thresholds(node):
    node.set_parameters([
        Parameter('unit2_load_high_current_a', value=0.0),
        Parameter('unit2_load_low_current_a', value=0.0),
    ])
    _leader_status(node, detection_ready=False)
    node.action = 'load'
    node.phase = 'contact'
    _action(node, 'load', roller=0.8)
    assert node.phase == 'error'
    assert node.reason == 'load_current_threshold_unset'


def test_unit1_can_report_its_own_completed_current_drop(node):
    node.set_parameters([
        Parameter('unit2_load_high_current_a', value=0.0),
        Parameter('unit2_load_low_current_a', value=0.0),
    ])
    node.action = 'load'
    node.phase = 'contact'
    _leader_status(node, detection_ready=True)
    _action(node, 'load', roller=0.8)
    node._tick()
    assert node.phase == 'loading'

    _leader_status(node, dropped=True, detection_ready=True)
    node._tick()
    assert node.phase == 'loaded'


def test_loading_continues_from_unit2_if_unit1_status_is_lost(node):
    node.action = 'load'
    node.phase = 'contact'
    _local_current(node, 1.0)
    _leader_status(node, 1.0)
    _action(node, 'load', roller=0.8)
    node.leader_status_time = time.monotonic() - 1.0
    node._tick()
    assert node.phase == 'loading'
    assert node.published_drive[-1].linear.x < 0.0


def test_loading_stops_when_both_detection_sources_are_lost(node):
    node.set_parameters([
        Parameter('unit2_load_high_current_a', value=0.0),
        Parameter('unit2_load_low_current_a', value=0.0),
    ])
    node.action = 'load'
    node.phase = 'contact'
    _leader_status(node, detection_ready=True)
    _action(node, 'load', roller=0.8)
    node.leader_status_time = time.monotonic() - 1.0
    node._tick()
    assert node.phase == 'error'
    assert node.reason == 'loading_current_detection_lost'
    assert node.published_drive[-1].linear.x == pytest.approx(0.0)
    assert node.published_roller[-1].data == pytest.approx(0.0)


def test_safety_stop_cancels_active_loading(node):
    node.action = 'load'
    node.phase = 'loading'
    node.loading_started = time.monotonic()
    node._on_stop(Bool(data=True))
    node._tick()
    assert node.phase == 'error'
    assert node.reason == 'safety_stop'
    assert node.published_drive[-1].linear.x == pytest.approx(0.0)
