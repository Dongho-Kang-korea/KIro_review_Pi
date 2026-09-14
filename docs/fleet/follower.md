# follower — `src/fleet/fleet/follower.py`

**최신화**: 2026-08-31

## 역할

1호기(리더)의 Fleet Protocol 명령을 수신해 이 호기(2 또는 3)의 미션 액션
(`fleet/action`)으로 변환하고, 5Hz로 자체 상태를 1호기에 보고한다. 명령
갱신이 끊기면 스스로 정지한다 — "명령 두절 시 정지" 원칙을 이 노드가 지킨다.

## 실행

`mars_launch/robot.launch.py`에서 `unit` 인자(2 또는 3)와 함께 뜬다.

## 설정

`src/fleet/config/follower.yaml` — `status_rate_hz`(기본
`protocol.STATUS_RATE_HZ`=5.0), `command_timeout_s`(기본
`protocol.CMD_TIMEOUT_S`=2.5, v1.2에서 1.0→2.5로 상향 — 1호기 leader의
1Hz 재발행 대비 1.0초는 빠듯하다, [TODO.md](../../TODO.md) 18번),
`state_timeout_s`(기본 2.5). 토픽명/JSON
규약 자체는 [protocol.md](protocol.md) 참고 — 1호기 `fleet/protocol.py`와의
계약이므로 한 호기만 바꾸지 않는다.

## 토픽

- 구독(절대경로, 규약 토픽): `/fleet/cmd/unit{unit}` (명령),
  `/fleet/state` (1호기 전역 상태), `/fleet/cargo/target` (화물 표적)
- 구독(이 호기 상대경로, 상태 취합용): `mission/phase`, `motor/status`,
  `marker/status`, `uwb/distance_mm`, `imu/heading_deg`, `roller_cmd`,
  `solenoid_status`, `cargo/red_detected`
- 발행: `fleet/action` (String JSON — arbiter가 구독), `safety/stop` (Bool),
  `/fleet/status/unit{unit}` (절대경로, 5Hz 상태 보고)

## 동작 요약

- **명령 검증**: `protocol.validate_command`로 걸러진 것만 받는다. `seq`가
  이전 것보다 크지 않으면 무시(중복/재전송 차단) — 단, `stop`은 항상 즉시
  반영.
- **명령 watchdog**(0.05s tick): `command_timeout_s` 안에 새 명령이 없으면
  자동 `stop` 발행. 단, `action`이 `idle`/`follow`이고 `state.section`이
  summer가 아니면 워치독을 요구하지 않는다(추종은 명령 대신 `/fleet/state`
  갱신으로 계속됨).
- **`/fleet/state` 기반 자동 전환**: `state.section != 'summer'`이고
  현재 액션이 idle/follow/stop이면, `phase`가 `finished`가 아닌 한 자동으로
  `follow` 액션을 발행한다 — 여름 외 계절은 개별 명령 없이 추종만 하기
  때문.
- 상태 보고에는 `_current()`(모터 중 최대 절대 전류)를 계산해 포함한다.

## 연결

- 위: 1호기 Fleet Protocol(`/fleet/*`) — 벤치 시험에서는 실물 1호기 대신
  [fake_leader](fake_leader.md)가 `/fleet/state`만 대신 흉내 낸다
  (`web_manual.launch.py`의 `start_fake_leader`, 기본 켜짐).
- 아래: `drive/arbiter`(`fleet/action` 구독), `mission/cargo_load`
  (`/fleet/cargo/target` 재구독), `ui/console`(`/fleet/status/unit{unit}`
  구독).

## 구현 상태

완성. 1호기 없이도(follower가 명령을 못 받는 상태로) 노드 자체는 정상
기동하며, 이 경우 arbiter가 `manual`/`auto` 모드로 계속 동작한다.

## 알려진 이슈

없음 — 발견되면 [TODO.md](../../TODO.md)에 등록한다.
