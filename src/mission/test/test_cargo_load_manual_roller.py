"""cargo_load.py의 웹 콘솔 수동 컨베이어(roller_manual_cmd) 병합 로직 검증
(2026-08-28 추가).

자동 미션이 idle/stop일 때만 수동값을 채택하고, 자동이 돌기 시작하면
즉시 자동이 우선하며, 콘솔이 갱신을 멈추면(timeout) 0으로 돌아가는지
확인한다.
"""
import time

import pytest
import rclpy
from std_msgs.msg import Float32, String

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


def _manual(node, value):
    msg = Float32(); msg.data = value
    node._on_roller_manual(msg)


def test_manual_roller_applied_when_idle(node):
    import json
    assert node.action == 'idle'          # 기본값
    received = {}

    def _capture(msg):
        received['payload'] = msg.data

    sub = node.create_subscription(String, 'mission/cargo/status', _capture, 10)
    _manual(node, 0.5)
    deadline = time.monotonic() + 2.0
    while 'payload' not in received and time.monotonic() < deadline:
        node._tick()
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    assert 'payload' in received, 'mission/cargo/status를 못 받았다'
    data = json.loads(received['payload'])
    assert data['roller'] == pytest.approx(0.5)
    assert data['roller_manual_active'] is True


def test_manual_roller_ignored_when_action_active(node):
    _manual(node, 0.5)
    node.action = 'approach'              # 자동 미션 진행 중
    node.phase = 'approach'
    node.roller_value = 0.0               # 자동 쪽은 0을 원하는 상태
    # roller_manual_cmd가 살아있어도 idle/stop이 아니므로 채택되면 안 된다.
    manual_fresh = (
        node.manual_roller is not None and node.manual_roller_time is not None and
        time.monotonic() - node.manual_roller_time <= float(
            node.get_parameter('manual_roller_timeout_s').value))
    assert manual_fresh is True
    accepted = node.action in ('idle', 'stop') and manual_fresh
    assert accepted is False


def test_manual_roller_expires_after_timeout(node):
    node.set_parameters([
        rclpy.parameter.Parameter('manual_roller_timeout_s', value=0.05)])
    _manual(node, 0.7)
    time.sleep(0.08)
    manual_fresh = (
        node.manual_roller is not None and node.manual_roller_time is not None and
        time.monotonic() - node.manual_roller_time <= float(
            node.get_parameter('manual_roller_timeout_s').value))
    assert manual_fresh is False


def test_manual_roller_clamped_to_unit_range(node):
    _manual(node, 5.0)
    assert node.manual_roller == pytest.approx(1.0)
    _manual(node, -5.0)
    assert node.manual_roller == pytest.approx(-1.0)


def test_safety_stop_zeroes_even_with_fresh_manual_command(node):
    import json
    received = {}

    def _capture(msg):
        received['payload'] = msg.data

    sub = node.create_subscription(String, 'mission/cargo/status', _capture, 10)
    _manual(node, 0.9)
    node.safety_stop = True
    deadline = time.monotonic() + 2.0
    while 'payload' not in received and time.monotonic() < deadline:
        node._tick()
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    assert 'payload' in received, 'mission/cargo/status를 못 받았다'
    data = json.loads(received['payload'])
    assert data['roller'] == pytest.approx(0.0)
