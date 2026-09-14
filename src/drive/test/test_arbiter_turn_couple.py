"""drive/arbiter.py — v1.2 turn/couple 액션 라우팅(TODO.md 18번).

turn/couple도 CARGO_ACTIONS에 속해 follow 모드에서 cargo 소스로 라우팅
되는지(16번 수정과 결합), _clean()이 angle_deg를 검증하는지 확인한다.
"""
import rclpy

import pytest

from drive.arbiter import ACTIONS, CARGO_ACTIONS, Arbiter


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Arbiter()
    yield n
    n.destroy_node()


def test_turn_and_couple_are_known_actions():
    assert 'turn' in ACTIONS
    assert 'couple' in ACTIONS


def test_turn_and_couple_are_cargo_actions():
    assert 'turn' in CARGO_ACTIONS
    assert 'couple' in CARGO_ACTIONS


def test_follow_mode_with_turn_or_couple_switches_to_cargo_source(node):
    for action in ('turn', 'couple'):
        node.mode = 'follow'
        node.published_action = {'action': action}
        assert node._source() == 'cargo'


def test_clean_accepts_turn_with_valid_angle():
    cleaned = Arbiter._clean(
        {'action': 'turn', 'seq': 7, 'angle_deg': 180.0}, 'fleet')
    assert cleaned == {
        'action': 'turn', 'origin': 'fleet', 'seq': 7, 'angle_deg': 180.0}


def test_clean_rejects_turn_without_angle():
    assert Arbiter._clean({'action': 'turn', 'seq': 8}, 'fleet') is None


def test_clean_rejects_turn_angle_out_of_range():
    assert Arbiter._clean(
        {'action': 'turn', 'seq': 9, 'angle_deg': 720.0}, 'fleet') is None


def test_clean_accepts_couple_without_extra_fields():
    cleaned = Arbiter._clean({'action': 'couple', 'seq': 10}, 'fleet')
    assert cleaned == {'action': 'couple', 'origin': 'fleet', 'seq': 10}


def test_locking_and_pull_verification_are_not_reported_as_locked(node):
    node.mode = 'follow'
    node.published_action = {'action': 'couple'}
    for phase in ('locking', 'verify_pull'):
        node.coupling_status = {'active': True, 'phase': phase}
        assert node._source() == 'coupling'
        assert node._phase() == 'contact'


def test_confirmed_locked_phase_persists_after_drive_source_changes(node):
    node.mode = 'follow'
    node.published_action = {'action': 'couple'}
    node.cargo_status = {'phase': 'idle'}
    node.coupling_status = {'active': False, 'phase': 'locked'}
    assert node._source() == 'cargo'
    assert node._phase() == 'locked'
