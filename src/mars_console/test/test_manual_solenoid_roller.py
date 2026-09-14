"""웹 콘솔의 솔레노이드/컨베이어/ArUco 축 오버레이 수동 조작 — 2호기에만
있고 1호기엔 없어야 하며(has_solenoid/has_roller/has_marker_axes),
set_*()가 그걸 지킨다 (2026-08-28 솔레노이드/컨베이어, 2026-08-29 ArUco
축 오버레이 추가).
"""
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


def test_units_2_3_have_solenoid_and_roller_topics_unit1_does_not():
    for unit in (2,):
        assert UNITS[unit]['solenoid_topic'] == f'/unit{unit}/solenoid_cmd'
        assert UNITS[unit]['roller_manual_topic'] == f'/unit{unit}/roller_manual_cmd'
        assert UNITS[unit]['marker_axes_topic'] == f'/unit{unit}/marker/axes_enable'
    assert 'solenoid_topic' not in UNITS[1]
    assert 'roller_manual_topic' not in UNITS[1]
    assert 'marker_axes_topic' not in UNITS[1]


def test_snapshot_flags_match_publisher_presence(node):
    for unit in UNITS:
        snap = node.snapshot(unit)
        assert snap['has_solenoid'] == (unit in node.solenoid_pub)
        assert snap['has_roller'] == (unit in node.roller_pub)
        assert snap['has_marker_axes'] == (unit in node.axes_pub)
    # 1호기는 셋 다 없어야 한다.
    assert node.snapshot(1)['has_solenoid'] is False
    assert node.snapshot(1)['has_roller'] is False
    assert node.snapshot(1)['has_marker_axes'] is False
    # 2호기는 셋 다 있어야 한다.
    assert node.snapshot(2)['has_solenoid'] is True
    assert node.snapshot(2)['has_roller'] is True
    assert node.snapshot(2)['has_marker_axes'] is True


def test_set_solenoid_rejects_unit_without_topic(node):
    assert node.set_solenoid(1, True) is False
    assert node.set_solenoid(2, True) is True


def test_set_marker_axes_rejects_unit_without_topic(node):
    assert node.set_marker_axes(1, True) is False
    assert node.set_marker_axes(2, True) is True


def test_set_roller_rejects_unit_without_topic_and_clamps(node):
    assert node.set_roller(1, 0.5) is False
    assert node.set_roller(2, 5.0) is True
    assert node.roller[2]['value'] == pytest.approx(1.0)
    assert node.set_roller(2, -5.0) is True
    assert node.roller[2]['value'] == pytest.approx(-1.0)


def test_drive_status_syncs_console_mode():
    """web_manual.launch.py의 start_mode=manual 불일치 버그 회귀 테스트."""
    import json
    from std_msgs.msg import String

    n = Console()
    try:
        assert n.mode[2] == 'idle'
        msg = String()
        msg.data = json.dumps({'mode': 'manual'})
        n._on_status(2, 'drive', msg)
        assert n.mode[2] == 'manual'
    finally:
        n.destroy_node()
