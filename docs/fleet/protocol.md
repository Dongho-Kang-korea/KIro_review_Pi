# protocol — `src/fleet/fleet/protocol.py`

**최신화**: 2026-08-31

## 역할

MARS 호기 간 통신 규약(Fleet Protocol)의 공통 정의 모듈이다. 독립 실행되는
노드가 아니라 `fleet/follower.py`(그리고 arbiter의 `ACTIONS` 사본)가 import
해서 쓰는 상수/검증 함수 모음. **토픽명과 허용 상태는 1호기 통신 코드와
항상 동일하게 유지해야 한다** — 한쪽만 고치면 호기 간 통신이 깨진다.

## 실행

노드 아님 — `fleet/follower`에 내장.

## 설정

없음(코드 상수). 값을 바꾸려면 이 파일과 1호기 쪽 대응 코드를 함께 고쳐야
한다.

## 정의 내용

- 토픽명: `TOPIC_STATE`(`/fleet/state`), `TOPIC_CARGO_TARGET`
  (`/fleet/cargo/target`), `TOPIC_CMD`(`/fleet/cmd/unit{}`),
  `TOPIC_STATUS`(`/fleet/status/unit{}`) — 전부 호기 네임스페이스 밖 절대
  경로, 호기별로 나뉜다. `TOPIC_RECEIVER_CHANNELS`(`/receiver/channels`,
  [rc_bridge](rc_bridge.md)가 구독)도 절대경로지만 **호기로 안 나뉜다** —
  1호기가 RC 채널 원시값을 통째로 방송하고, 구독 쪽(rc_bridge)이 채널
  안의 호기 선택 스위치(CH7)로 스스로 자기 몫을 걸러낸다. 2026-08-30
  1호기 쪽 실제 코드를 확인하고 맞춘 이름이다([TODO.md](../../TODO.md)
  14번 경위).
- `SECTIONS`: spring/summer/autumn/winter/escort
- `ACTIONS`: idle/follow/approach/turn/load/hold/couple/release/stop
  (`drive/arbiter.py`의 `ACTIONS`와 반드시 같아야 한다 — arbiter는 fleet에
  빌드 의존하지 않으려고 이 목록을 자체 복사해 두고 있다). `turn`
  (`angle_deg` 동반, -360~360 검증)과 `couple`은 v1.2에서 추가됐다
  ([TODO.md](../../TODO.md) 18번) — 둘 다 1호기가 여름 적재 단계를 직접
  지휘할 때 쓰는 명령이고, 처리는 [cargo_load](../mission/cargo_load.md)
  참고.
- `PHASES`: idle/following/approach/search/turning/backing/contact/
  loading/loaded/locked/error
- `CMD_TIMEOUT_S`(2.5 — v1.2에서 1.0→2.5로 늘렸다. 1호기 leader의 명령
  재발행이 1Hz라 1.0초는 정상 동작 중에도 아슬아슬하게 걸린다),
  `STATUS_RATE_HZ`(5.0)
- 검증 함수: `decode`(JSON + 필수 `t` 검사), `validate_command`
  (action/seq/roller 검사), `validate_state`(section/mode 검사),
  `validate_cargo_target`(숫자 필드 강제 변환), `message`(공통 envelope
  생성 헬퍼)

## 연결

- 위: 없음(공용 정의).
- 아래: `fleet/follower.py`(전체 사용), `fleet/rc_bridge.py`(`TOPIC_RC`만
  사용), `drive/arbiter.py`(`ACTIONS`만 수동 복사, import는 안 함).

## 구현 상태

완성.

## 알려진 이슈

`drive/arbiter.py`의 `ACTIONS` 사본과 이 파일의 `ACTIONS`가 어긋나지 않게
유지하는 책임은 사람/에이전트에게 있다 — 자동 동기화 장치가 없다. 둘 중
하나를 고칠 때 반드시 다른 쪽도 확인한다.
