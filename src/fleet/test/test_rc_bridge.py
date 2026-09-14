"""fleet/rc_bridge.py — 1호기 /receiver/channels(14채널, 1000~2000) 처리.

2026-08-30 채널 재배치(CH1/2 주행, CH3 솔레노이드, CH5 VRA, CH7 호기 선택,
CH8 정지/재생, CH9 컨베이어, CH10 자동/수동)와 "CH7 이탈 시 유지, 토픽
자체가 끊길 때만 0" 정책, 웹 rc/enabled 토글의 "즉시 0" 정책을 검증한다.
"""
import time

import rclpy

import pytest

from fleet.rc_bridge import RcBridge


@pytest.fixture(scope='module', autouse=True)
def _rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = RcBridge()
    yield n
    n.destroy_node()


def _channels(steer=1500, throttle=1500, solenoid=1500, vra=2000, select=2000,
              stop=2000, conveyor=1500, mode=1000):
    """14채널 배열을 만든다. 기본값: 중립 스틱, VRA 100%, 2호기 선택,
    재생 상태, 컨베이어 중립, CH10=수동(rc)."""
    from std_msgs.msg import UInt16MultiArray
    ch = [1500] * 14
    ch[0], ch[1], ch[2] = steer, throttle, solenoid
    ch[4] = vra
    ch[6], ch[7], ch[8], ch[9] = select, stop, conveyor, mode
    msg = UInt16MultiArray()
    msg.data = ch
    return msg


def _enable(node):
    from std_msgs.msg import Bool
    node._on_enabled(Bool(data=True))


# ---------- 순수 변환 함수 ----------

def test_default_topic_falls_back_to_protocol_constant(node):
    assert node.topic == '/receiver/channels'


def test_normalize_center_and_extremes(node):
    assert node._normalize(1500.0, reversed_=False) == 0.0
    assert node._normalize(2000.0, reversed_=False) == pytest.approx(1.0)
    assert node._normalize(1000.0, reversed_=False) == pytest.approx(-1.0)
    assert node._normalize(2000.0, reversed_=True) == pytest.approx(-1.0)


def test_knob_scale_matches_unit1_formula(node):
    assert node._knob_scale(1000.0) == pytest.approx(0.0)
    assert node._knob_scale(2000.0) == pytest.approx(1.0)
    assert node._knob_scale(1500.0) == pytest.approx(0.5)
    assert node._knob_scale(500.0) == pytest.approx(0.0)   # 클램프
    assert node._knob_scale(3000.0) == pytest.approx(1.0)  # 클램프


def test_conveyor_value(node):
    assert node._conveyor_value(1000) == -1.0
    assert node._conveyor_value(1500) == 0.0
    assert node._conveyor_value(2000) == 1.0
    assert node._conveyor_value(1250) == 0.0   # 경계 포함(< 아님)
    assert node._conveyor_value(1750) == 0.0


def test_solenoid_locked_direction(node):
    assert node._solenoid_locked(1000) is True   # 내림 = 잠금
    assert node._solenoid_locked(2000) is False  # 올림 = 해제
    node.solenoid_reversed = True
    assert node._solenoid_locked(1000) is False


# ---------- rc/enabled 게이트 ----------

def test_disabled_by_default_ignores_channels(node):
    published = []
    node.cmd_pub.publish = published.append
    node._on_channels(_channels())
    assert published == []


def test_enable_then_disable_zeroes_immediately(node):
    published = []
    node.cmd_pub.publish = published.append
    roller_published = []
    node.roller_pub.publish = roller_published.append

    _enable(node)
    node._on_channels(_channels(steer=2000))
    assert published[-1].angular.z != 0.0

    from std_msgs.msg import Bool
    node._on_enabled(Bool(data=False))
    assert published[-1].linear.x == 0.0 and published[-1].angular.z == 0.0
    assert roller_published[-1].data == 0.0

    published.clear()
    node._on_channels(_channels(steer=2000))
    assert published == []  # 꺼진 뒤엔 채널이 와도 무시


# ---------- CH7(호기 선택) / "유지" ----------

def test_not_selected_holds_last_output(node):
    published = []
    node.cmd_pub.publish = published.append
    _enable(node)

    node._on_channels(_channels(steer=2000, select=2000))  # 선택됨, 조향 최대
    first = published[-1].angular.z
    assert first == pytest.approx(node.max_angular)

    # CH7이 1호기로 넘어가고, 스틱도 중립으로 돌아왔다고 가정해도
    # "마지막 값"을 그대로 유지해야 한다(0으로 바뀌면 안 됨).
    node._on_channels(_channels(steer=1500, select=1000))
    held = published[-1].angular.z
    assert held == pytest.approx(first)


def test_gap_band_counts_as_not_selected(node):
    # 1350~1649 애매 구간은 '2호기 아님'으로 처리돼야 한다(안전 편향).
    published = []
    node.cmd_pub.publish = published.append
    _enable(node)
    node._on_channels(_channels(steer=2000, select=1500))  # 애매 구간
    assert not node.robot_selected
    assert published[-1].angular.z == 0.0  # 아직 아무 값도 없어 기본 0 유지


# ---------- CH8(정지/재생) ----------

def test_stop_gate_zeroes_drive_but_not_solenoid(node):
    published = []
    node.cmd_pub.publish = published.append
    solenoid_published = []
    node.solenoid_pub.publish = solenoid_published.append
    _enable(node)

    node._on_channels(_channels(steer=2000, solenoid=1000, select=2000, stop=2000))
    assert published[-1].angular.z != 0.0
    assert solenoid_published[-1].data is True  # 잠금

    solenoid_published.clear()
    node._on_channels(_channels(steer=2000, solenoid=2000, select=2000, stop=1000))
    assert published[-1].angular.z == 0.0        # 정지 -> 주행 0
    assert solenoid_published == []              # 솔레노이드는 안 건드림(값 불변 유지)


# ---------- CH9(컨베이어) ----------

def test_conveyor_channel_drives_roller_manual_cmd(node):
    roller_published = []
    node.roller_pub.publish = roller_published.append
    _enable(node)
    node._on_channels(_channels(select=2000, stop=2000, conveyor=2000))
    assert roller_published[-1].data == pytest.approx(1.0)


# ---------- CH10(자동/수동) 원격 모드 전환 ----------

def test_mode_channel_sets_rc_then_auto(node):
    mode_published = []
    node.mode_pub.publish = mode_published.append
    _enable(node)
    node._on_channels(_channels(select=2000, stop=2000, mode=1000))
    assert mode_published[-1].data == 'rc'

    node._on_channels(_channels(select=2000, stop=2000, mode=2000))
    assert mode_published[-1].data == 'auto'


def test_mode_channel_ignored_when_not_selected(node):
    mode_published = []
    node.mode_pub.publish = mode_published.append
    _enable(node)
    node._on_channels(_channels(select=1000, mode=2000))
    assert mode_published == []


# ---------- VRA 속도 스케일 ----------

def test_speed_scale_multiplies_output(node):
    published = []
    node.cmd_pub.publish = published.append
    _enable(node)
    node._on_channels(_channels(steer=2000, select=2000, stop=2000, vra=1500))
    assert published[-1].angular.z == pytest.approx(node.max_angular * 0.5)


# ---------- 짧은/개수 다른 프레임 ----------

def test_short_frame_dropped_without_raising(node):
    from std_msgs.msg import UInt16MultiArray
    msg = UInt16MultiArray()
    msg.data = [1500] * 3  # mode_channel(10)까지 못 미침
    _enable(node)
    node._on_channels(msg)  # 예외 없이 조용히 버려야 한다


# ---------- 워치독: 신호 두절 시 솔레노이드 안전 복귀 ----------

def test_watchdog_force_locks_solenoid_after_signal_loss(node):
    solenoid_published = []
    node.solenoid_pub.publish = solenoid_published.append
    # signal_timeout_s는 __init__에서 한 번만 읽어 캐시해 두는 값이라
    # (arbiter.source_timeout_s처럼 매 tick 다시 안 읽음) set_parameters로는
    # 안 바뀐다 — 속성을 직접 덮어써야 한다.
    node.signal_timeout_s = 0.05
    _enable(node)

    node._on_channels(_channels(solenoid=2000, select=2000, stop=2000))  # 해제 상태로 둠
    assert node.last_solenoid_locked is False

    time.sleep(0.1)
    node._watchdog()
    assert solenoid_published[-1].data is True
    assert node.last_solenoid_locked is True


def test_watchdog_noop_when_disabled(node):
    solenoid_published = []
    node.solenoid_pub.publish = solenoid_published.append
    node._watchdog()
    assert solenoid_published == []


# ---------- 상태 발행 ----------

def test_status_reports_enabled_and_selection(node):
    import json
    published = []
    node.status_pub.publish = published.append
    _enable(node)
    node._on_channels(_channels(select=2000, stop=2000))
    node._publish_status()
    payload = json.loads(published[-1].data)
    assert payload['enabled'] is True
    assert payload['robot_selected'] is True
    assert payload['run_active'] is True
