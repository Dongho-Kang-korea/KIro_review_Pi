"""drive/arbiter.py — follow 모드에서 화물 명령이 오면 cargo 소스로
바뀌는지 확인한다(TODO.md 16번).

예전엔 mode=='follow'면 _source()가 무조건 'follow'를 반환해서,
mission/action은 정상 갱신되는데 cmd_vel/cargo가 있어도 안 쓰이고 실제
모터는 안 움직이는 버그가 있었다 — ~/ros2_ws(팀원의 별도 워크스페이스)
쪽 코드와 비교해 발견했다.
"""
import rclpy

import pytest

from drive.arbiter import CARGO_ACTIONS, Arbiter


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


def test_follow_mode_with_no_cargo_action_still_uses_follow_source(node):
    node.mode = 'follow'
    node.published_action = {'action': 'idle'}
    assert node._source() == 'follow'


@pytest.mark.parametrize('action', CARGO_ACTIONS)
def test_follow_mode_with_cargo_action_switches_to_cargo_source(node, action):
    node.mode = 'follow'
    node.published_action = {'action': action}
    assert node._source() == 'cargo'


def test_follow_mode_with_follow_action_uses_follow_source(node):
    node.mode = 'follow'
    node.published_action = {'action': 'follow'}
    assert node._source() == 'follow'


def test_coupling_still_overrides_follow_and_cargo(node):
    # 결합은 모드/액션보다 항상 위여야 한다 — 회귀 확인.
    node.mode = 'follow'
    node.published_action = {'action': 'load'}
    node.coupling_status = {'phase': 'locking'}
    assert node._source() == 'coupling'
