"""drive/arbiter.py — 'rc' 모드 추가(TODO.md 14번).

follow와 달리 진입 게이트가 없다(manual과 동일 취급)는 설계를, 그리고
rc_link 상태가 순수 표시용으로 arbiter의 기존 source_timeout_s 판정을
그대로 재사용하는지를 검증한다.
"""
import time

import rclpy
from rclpy.parameter import Parameter

import pytest

from drive.arbiter import MODES, Arbiter


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


def test_rc_is_a_known_mode():
    assert 'rc' in MODES


def test_rc_source_registered(node):
    assert node.SOURCES['rc'] == 'cmd_vel/rc'


def test_rc_mode_entry_is_not_gated_unlike_follow(node):
    # follow는 fleet_alive() 없이는 _on_mode()가 거부하지만, rc는 manual과
    # 똑같이 즉시 들어갈 수 있어야 한다(1호기 RC 신호 유무와 무관).
    from std_msgs.msg import String
    assert not node.rc_alive()
    node._on_mode(String(data='rc'))
    assert node.mode == 'rc'


def test_source_returns_rc_when_mode_is_rc(node):
    node.mode = 'rc'
    assert node._source() == 'rc'


def test_rc_alive_reuses_source_timeout(node):
    node.set_parameters([Parameter('source_timeout_s', value=0.2)])
    assert node.rc_alive() is False
    node.received['rc'] = time.monotonic()
    assert node.rc_alive() is True
    node.received['rc'] = time.monotonic() - 1.0
    assert node.rc_alive() is False


def test_status_reports_rc_link(node):
    import json
    captured = []
    node.status_pub.publish = lambda msg: captured.append(json.loads(msg.data))
    node._publish_status()
    assert 'rc_link' in captured[0]
    assert captured[0]['rc_link'] is False
