"""MARS 호기 통신 규약 1.2의 공통 정의.

v1.2 변경(2026-08-31, TODO.md 18번 — 1호기 `mars1-rokacup26`의
`comp-upgrade-20260830` 브랜치 기준, 아직 그쪽도 origin에 push 전):
  - action 추가: ``turn``(``angle_deg`` 동반, 임의 각도 회전), ``couple``
    (자동 결합 시작 — 이 저장소는 ``turn``으로 미리 방향을 잡은 뒤 별도로
    보내는 걸 전제한다, ``docs/mission/cargo_load.md`` 참고).
  - 명령 감시선(``CMD_TIMEOUT_S``)을 1.0→2.5초로 늘렸다. 1호기 leader의
    재발행이 1Hz라 1.0초는 정상 동작 중에도 아슬아슬하게 걸린다.
"""
import json
import time

# 토픽명과 허용 상태는 1호기 통신 코드와 항상 동일하게 유지한다.
TOPIC_STATE = '/fleet/state'
TOPIC_CARGO_TARGET = '/fleet/cargo/target'
TOPIC_CMD = '/fleet/cmd/unit{}'
TOPIC_STATUS = '/fleet/status/unit{}'
# RC(무선 조종기) 채널 — 위 4개(호기별 /fleet/*)와 달리 호기로 안 나뉜다.
# 1호기가 RC 수신기 원시 채널을 그대로 방송하고(UInt16MultiArray, 14개),
# 이 채널의 CH7(호기 선택)을 보는 쪽(fleet/rc_bridge.py)이 스스로 자기
# 몫인지 걸러낸다. 2026-08-30 1호기 쪽 실제 코드를 확인하고 맞춘 이름 —
# 더 이상 임시 추측값이 아니다(TODO.md 14번 경위 참고).
TOPIC_RECEIVER_CHANNELS = '/receiver/channels'

SECTIONS = {'spring', 'summer', 'autumn', 'winter', 'escort'}
ACTIONS = {'idle', 'follow', 'approach', 'turn', 'load', 'hold',
           'couple', 'release', 'stop'}
PHASES = {
    'idle', 'following', 'approach', 'search', 'turning', 'backing',
    'contact', 'loading', 'loaded', 'locked', 'error',
}
CMD_TIMEOUT_S = 2.5
STATUS_RATE_HZ = 5.0


class ProtocolError(ValueError):
    pass


def decode(payload):
    try:
        value = json.loads(payload)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f'invalid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise ProtocolError('message must be a JSON object')
    if not isinstance(value.get('t'), (int, float)):
        raise ProtocolError('missing numeric t')
    return value


def validate_command(value):
    action = str(value.get('action', '')).lower()
    if action not in ACTIONS:
        raise ProtocolError(f'unknown action: {action}')
    seq = value.get('seq')
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise ProtocolError('seq must be a non-negative integer')
    result = dict(value, action=action, seq=seq)
    if action == 'load':
        roller = result.get('roller')
        if not isinstance(roller, (int, float)) or not -1.0 <= float(roller) <= 1.0:
            raise ProtocolError('load requires roller in -1.0..1.0')
        result['roller'] = float(roller)
    if action == 'turn':
        angle = result.get('angle_deg')
        if not isinstance(angle, (int, float)) or not -360.0 <= float(angle) <= 360.0:
            raise ProtocolError('turn requires angle_deg in -360..360')
        result['angle_deg'] = float(angle)
    return result


def validate_state(value):
    section = str(value.get('section', '')).lower()
    mode = str(value.get('mode', '')).lower()
    if section not in SECTIONS:
        raise ProtocolError(f'unknown section: {section}')
    if mode not in {'auto', 'manual'}:
        raise ProtocolError(f'unknown mode: {mode}')
    return dict(value, section=section, mode=mode,
                leader_stopped=bool(value.get('leader_stopped', False)))


def validate_cargo_target(value):
    result = dict(value, detected=bool(value.get('detected', False)))
    for key in ('distance_mm', 'lateral_mm', 'confidence'):
        if key in result and result[key] is not None:
            try:
                result[key] = float(result[key])
            except (TypeError, ValueError) as exc:
                raise ProtocolError(f'{key} must be numeric') from exc
    return result


def message(**values):
    return json.dumps({'t': time.time(), **values}, separators=(',', ':'))
