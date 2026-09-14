"""cargo_load.py의 approach/turn 180도 회전과 couple 전달 검증."""
import json

import rclpy
from std_msgs.msg import Float32, String

import pytest

from mission.summer.cargo_load import CargoLoad


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = CargoLoad()
    yield n
    n.destroy_node()


def _heading(node, deg):
    node._on_heading(Float32(data=deg))


def _action(node, payload):
    node._on_action(String(data=json.dumps(payload)))


def test_turn_command_sets_target_from_angle_deg_and_starts_turning(node):
    _heading(node, 10.0)
    _action(node, {'action': 'turn', 'seq': 1, 'angle_deg': 90.0})
    assert node.phase == 'turning'
    assert node.turn_target_deg == pytest.approx(90.0)
    assert node.turn_origin is None
    assert node.turn_next_phase == 'idle'

    node.action = 'turn'
    node._tick()
    assert node.turn_origin == pytest.approx(10.0)


def test_turn_command_negative_angle_is_taken_as_magnitude(node):
    # 회전 방향은 angular.z 부호가 아니라 turn_speed_rps 하나로 고정
    # 회전한다(기존 로직 그대로) — angle_deg는 크기로만 쓴다.
    _action(node, {'action': 'turn', 'seq': 1, 'angle_deg': -120.0})
    assert node.turn_target_deg == pytest.approx(120.0)


def test_turn_via_command_completes_to_idle_not_backing(node):
    node.action = 'turn'
    _heading(node, 0.0)
    _action(node, {'action': 'turn', 'seq': 1, 'angle_deg': 90.0})
    node._tick()  # fresh heading 0도를 회전 원점으로 저장
    # turn_tolerance_deg 기본값 8.0 -> 목표(90)-8=82도 이상이면 완료.
    _heading(node, 95.0)
    node._tick()
    assert node.phase == 'idle'


def test_approach_immediately_turns_and_never_enters_backing(node):
    _heading(node, 0.0)
    _action(node, {'action': 'approach', 'seq': 1})
    assert node.phase == 'turning'
    assert node.turn_next_phase == 'approach'
    assert node.turn_target_deg == pytest.approx(180.0)

    node._tick()  # fresh heading 0도를 회전 원점으로 저장
    _heading(node, 175.0)  # 180 - 8(기본 tolerance) = 172도 이상
    node._tick()
    assert node.phase == 'approach'


def test_couple_command_is_not_routed_by_cargo_load(node):
    """결합 라우팅은 2026-09-03 에 coupling.py 로 옮겼다(TODO 24번).

    결합은 겨울에도 쓰는데 예전에는 여름 노드인 cargo_load 를 거쳐야
    했다 — 계절별로 노드를 게이팅하면 겨울 결합이 끊긴다. 이제
    coupling.py 가 fleet/action 을 직접 구독하므로, 이 노드는 couple 을
    받아도 아무것도 하지 않아야 한다(화물 단계도 건드리지 않는다).
    """
    assert not hasattr(node, 'coupling_pub')
    node.phase = 'idle'
    _action(node, {'action': 'couple', 'seq': 1})
    assert node.phase == 'idle'
    assert node.roller_value == 0.0


def test_turn_command_ignored_outside_allowed_phases(node):
    node.phase = 'loading'  # 진행 중인 다른 단계 — turn을 안 받는다
    _action(node, {'action': 'turn', 'seq': 1, 'angle_deg': 90.0})
    assert node.phase == 'loading'
