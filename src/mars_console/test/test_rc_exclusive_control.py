"""웹/RC 조종 방식을 배타적 전권으로 강화 (TODO.md 21번).

지금까지는 웹→RC 차단만 완전했다(rc_bridge.py가 enabled=False면 채널을
통째로 무시). RC→웹 방향은 "모드" 버튼만 unit.html에서 UI로 잠갔을 뿐
서버는 막지 않아, RC가 전권을 쥔 동안에도 웹 API를 직접 호출하면
solenoid_cmd/roller_manual_cmd/drive/mode_cmd에서 rc_bridge와 경쟁 상태가
생길 수 있었다.

이 테스트는 새로 추가한 `Console.rc_owns()`가:
  1. `rc/status.enabled`가 fresh(=stale_seconds 안)하게 참일 때만 참이고,
  2. rc_bridge가 죽어 피드백이 끊기면(stale) 자동으로 거짓으로 풀리며
     (락 영구 고착 방지 — 안전 기본값은 웹),
  3. 1호기처럼 rc 자체가 없는 호기는 항상 거짓이고,
를 검증하고, 이 값이 실제로 두 지점에서 제대로 쓰이는지 확인한다:
  - `_tick_drive()` — RC 전권이면 cmd_vel/manual·roller_manual_cmd를
    아예 재발행하지 않는다(API만 막는 것으로는 부족 — 이미 쥐고 있던
    값이 input_timeout까지 계속 나갈 수 있어서).
  - `api_mode`/`api_drive`/`api_solenoid`/`api_roller` — RC 전권이면
    409로 거부하고, `api_estop`/`api_rc_enabled`는 예외로 항상 통과한다
    (안전 정지와 조종 방식 전환 자체는 막으면 안 된다).
"""
import time

import rclpy

import pytest

import mars_console.console as console_mod
from mars_console.console import Console, UNITS


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


def _set_rc_feed(node, unit, enabled, age_s=0.0):
    with node.lock:
        node.feeds[unit]['rc'] = (
            {'enabled': enabled}, time.monotonic() - age_s)


# ---------- rc_owns() ----------

def test_rc_owns_false_by_default(node):
    assert node.rc_owns(2) is False


def test_rc_owns_true_when_fresh_and_enabled(node):
    _set_rc_feed(node, 2, True)
    assert node.rc_owns(2) is True


def test_rc_owns_false_when_enabled_false(node):
    _set_rc_feed(node, 2, False)
    assert node.rc_owns(2) is False


def test_rc_owns_false_when_stale():
    """rc_bridge가 죽어 rc/status가 끊기면 stale_seconds 뒤 자동으로
    풀려야 한다 — 웹이 영구히 잠기면 안 된다(안전 기본값은 웹)."""
    n = Console()
    try:
        _set_rc_feed(n, 2, True, age_s=n.stale + 1.0)
        assert n.rc_owns(2) is False
    finally:
        n.destroy_node()


def test_rc_owns_false_for_unit_without_rc(node):
    # 1호기는 rc 피드 자체가 없다 — feeds[1]에 'rc' 키가 생길 일이 없다.
    assert 'rc' not in UNITS[1]['status_topics']
    assert node.rc_owns(1) is False


# ---------- _tick_drive() 재발행 게이팅 ----------

def test_tick_drive_skips_publish_when_rc_owns(node):
    unit = 2
    node.mode[unit] = 'manual'
    with node.lock:
        node.drive[unit] = {'linear': 0.2, 'angular': 0.1, 'time': time.monotonic()}
        node.roller[unit] = {'value': 0.5, 'time': time.monotonic()}
    _set_rc_feed(node, unit, True)

    cmd_seen, roller_seen = [], []
    node.cmd_pub[unit].publish = cmd_seen.append
    node.roller_pub[unit].publish = roller_seen.append

    node._tick_drive()

    assert cmd_seen == []
    assert roller_seen == []


def test_tick_drive_publishes_when_rc_not_owns(node):
    unit = 2
    node.mode[unit] = 'manual'
    with node.lock:
        node.drive[unit] = {'linear': 0.2, 'angular': 0.1, 'time': time.monotonic()}
        node.roller[unit] = {'value': 0.5, 'time': time.monotonic()}
    # rc 피드 없음(기본 웹 소유) — 정상적으로 재발행돼야 한다.

    cmd_seen, roller_seen = [], []
    node.cmd_pub[unit].publish = cmd_seen.append
    node.roller_pub[unit].publish = roller_seen.append

    node._tick_drive()

    assert len(cmd_seen) == 1
    assert cmd_seen[0].linear.x == pytest.approx(0.2)
    assert len(roller_seen) == 1
    assert roller_seen[0].data == pytest.approx(0.5)


# ---------- API 레벨 409 거부 ----------

@pytest.fixture
def client(node):
    console_mod.ros_node = node
    with console_mod.app.test_client() as c:
        yield c
    console_mod.ros_node = None


def test_api_mode_blocked_when_rc_owns(node, client):
    _set_rc_feed(node, 2, True)
    res = client.post('/api/unit/2/mode', json={'mode': 'manual'})
    assert res.status_code == 409
    assert res.get_json()['ok'] is False


def test_api_drive_blocked_when_rc_owns(node, client):
    _set_rc_feed(node, 2, True)
    res = client.post('/api/unit/2/drive', json={'linear': 0.1, 'angular': 0.0})
    assert res.status_code == 409


def test_api_solenoid_blocked_when_rc_owns(node, client):
    _set_rc_feed(node, 2, True)
    res = client.post('/api/unit/2/solenoid', json={'lock': True})
    assert res.status_code == 409


def test_api_roller_blocked_when_rc_owns(node, client):
    _set_rc_feed(node, 2, True)
    res = client.post('/api/unit/2/roller', json={'value': 0.5})
    assert res.status_code == 409


def test_api_mode_allowed_when_rc_not_owns(node, client):
    res = client.post('/api/unit/2/mode', json={'mode': 'manual'})
    assert res.status_code == 200
    assert res.get_json()['ok'] is True


def test_api_estop_never_blocked_by_rc(node, client):
    """비상 정지는 조종 전권과 무관하게 항상 통과해야 한다(안전 예외)."""
    _set_rc_feed(node, 2, True)
    res = client.post('/api/unit/2/estop', json={'stop': True})
    assert res.status_code == 200


def test_api_rc_enabled_never_blocked_by_rc(node, client):
    """조종 방식 토글 자체가 막히면 RC에서 웹으로 되돌아올 방법이
    없어진다 — 반드시 항상 통과해야 한다."""
    _set_rc_feed(node, 2, True)
    res = client.post('/api/unit/2/rc/enabled', json={'enabled': False})
    assert res.status_code == 200


def test_api_mode_blocked_only_for_unit_currently_owned(node, client):
    """1호기는 RC 전권이 아니므로 2호기가 잠겨 있어도 그대로 통과해야
    한다 — 잠금은 호기별로 독립적이다.

    예전에는 3호기로 확인했는데 3호기 지원이 제거되면서(TODO 19) 404 가
    났다. RC 가 없는 1호기로 같은 성질을 확인한다."""
    _set_rc_feed(node, 2, True)
    res = client.post('/api/unit/1/mode', json={'mode': 'manual'})
    assert res.status_code == 200
