"""웹 콘솔 — 2호기 'rc'(조종기) 모드 노출(TODO.md 14번).

follow_available과 달리 rc_available은 모드 진입을 잠그는 데 안 쓰인다
(arbiter가 게이트를 안 두므로) — 여기서는 snapshot이 arbiter의
drive/status.rc_link를 그대로 반영하는지만 확인한다.

2026-08-30: 웹/RC 조종 방식 토글(`rc/enabled`)이 추가됐다 — snapshot의
`rc_enabled`/`rc_robot_selected`/`rc_run_active`는 콘솔 자체 기억이
아니라 rc_bridge가 되돌려주는 `rc/status`를 그대로 반영해야 한다
(marker/status.axes_enabled와 같은 반영 패턴).
"""
import time

import rclpy

import pytest

from mars_console.console import UNITS, Console


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = Console()
    yield n
    n.destroy_node()


def test_units_2_3_expose_rc_mode_unit1_does_not():
    for unit in (2,):
        assert 'rc' in UNITS[unit]['modes']
    assert 'rc' not in UNITS[1]['modes']


def test_snapshot_rc_available_follows_drive_status_rc_link(node):
    unit = 2
    with node.lock:
        node.feeds[unit]['drive'] = ({'rc_link': True}, time.monotonic())
    assert node.snapshot(unit)['rc_available'] is True

    with node.lock:
        node.feeds[unit]['drive'] = ({'rc_link': False}, time.monotonic())
    assert node.snapshot(unit)['rc_available'] is False


def test_snapshot_rc_available_defaults_false_without_drive_status(node):
    assert node.snapshot(2)['rc_available'] is False


def test_unit2_has_rc_enabled_topic_unit1_does_not():
    for unit in (2,):
        assert UNITS[unit]['rc_enabled_topic'] == f'/unit{unit}/rc/enabled'
    assert 'rc_enabled_topic' not in UNITS[1]


def test_has_rc_matches_publisher_presence(node):
    assert node.snapshot(1)['has_rc'] is False
    assert node.snapshot(2)['has_rc'] is True


def test_set_rc_enabled_rejects_unit_without_topic(node):
    assert node.set_rc_enabled(1, True) is False
    assert node.set_rc_enabled(2, True) is True


def test_snapshot_rc_enabled_reflects_rc_status_not_local_echo(node):
    """set_rc_enabled()는 요청만 보낸다 — snapshot의 rc_enabled는
    rc_bridge가 rc/status로 되돌려준 값만 본다(요청 자체를 기억해 바로
    true로 보여주면 안 된다, 3축 오버레이 토글과 동일한 원칙)."""
    unit = 2
    node.set_rc_enabled(unit, True)
    assert node.snapshot(unit)['rc_enabled'] is False  # 아직 rc/status가 안 옴

    with node.lock:
        node.feeds[unit]['rc'] = (
            {'enabled': True, 'robot_selected': True, 'run_active': False},
            time.monotonic())
    snap = node.snapshot(unit)
    assert snap['rc_enabled'] is True
    assert snap['rc_robot_selected'] is True
    assert snap['rc_run_active'] is False
