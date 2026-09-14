# arbiter — `src/drive/drive/arbiter.py`

**최신화**: 2026-09-04 (2026-09-01 `coupled` 모드 추가, `/fleet/cmd_vel/unit2` 독립 속도 토픽 반영)

## 역할

이 호기의 **운용 모드를 소유**하고, 여러 속도 후보(manual/follow/cargo/
coupling) 중 하나를 최종 `cmd_vel`로 선택해 발행하는 유일한 노드다. 동시에
`fleet/action`(1호기)과 `mission/request`(UI) 중 모드에 맞는 것 하나를
골라 `mission/action`으로 재발행해, 미션 노드들이 명령 출처를 신경 쓰지
않게 한다.

## 실행

`mars_launch/robot.launch.py`(`season` 인자와 함께), `manual.launch.py`/
`manual_drive.launch.py`/`web_manual.launch.py`(전부 `start_mode: manual`
고정)에서 뜬다.

## 설정

`src/drive/config/arbiter.yaml` — `publish_rate_hz`(기본 50), `source_timeout_s`
(기본 0.5, 후보 소스가 이 시간 안 갱신되면 버림 — `rc_link` 판정도 이
값을 재사용한다), `start_mode`(기본 idle), `fleet_state_timeout_s`(기본
2.5, 추종 중 1호기 두절 판정). `manual` 구역에 키보드/웹 조작 상한
(`max_linear`/`max_angular` 등)도 여기 있다.

## 토픽

- 구독: `cmd_vel/follow`, `cmd_vel/cargo`, `cmd_vel/coupling`,
  `cmd_vel/coupled`(2026-09-01 추가), `cmd_vel/manual`, `cmd_vel/rc`
  (Twist 후보 6종), `fleet/action` (1호기
  미션 명령), `mission/request` (UI 미션 요청), `/fleet/state` (절대경로,
  1호기 생존 판정), `safety/stop`, `safety/emergency_stop` (Bool),
  `drive/mode_cmd` (String — idle/manual/auto/follow/rc/reset_emergency),
  `mission/follow/status`, `mission/cargo/status`, `mission/coupling/status`
- 발행: `cmd_vel` (Twist, **이 노드만 발행**), `drive/status` (String JSON —
  mode/season/action/source/phase/fleet_link/rc_link/safety_stop/
  emergency_stop/coupling_phase 등), `mission/phase` (String), `mission/action`
  (String JSON, TRANSIENT_LOCAL QoS — 늦게 뜬 미션 노드도 마지막 명령을 받음)

## 동작 요약

**모드**(`drive/mode_cmd`로 런타임 전환, 시작은 항상 `idle`):

| 모드 | 주행 소스 | 미션 명령 출처 | 1호기 필요 |
|---|---|---|---|
| `idle` | 없음(0) | 없음 | 아니오 |
| `manual` | `cmd_vel/manual` | 없음(인식 노드는 계속 돎) | 아니오 |
| `auto` | 미션 노드 | `mission/request`(UI) | 아니오 |
| `follow` | (아래 참고) | `/fleet/cmd/unitN`→`fleet/action`(1호기) | **예** |
| `coupled` | `cmd_vel/coupled`(`mission/coupled_drive`) | 없음(1호기 속도 `/fleet/cmd_vel/unit2` 복제만) | **예** |
| `rc` | `cmd_vel/rc`(`fleet/rc_bridge`) | 없음 | 아니오(신호는 1호기 경유) |

- **`coupled`(2026-09-01 추가)**: 결합(`locked`) 완료 후 1호기와 같은
  속도로 굴러가는 협조 주행 전용 모드다 — `coupling`은 결합하는
  '과정', `coupled`는 결합이 끝난 '이후'를 가리킨다. `follow`와 똑같이
  `fleet_alive()`가 아니면 진입을 거부하고, 진입 후 끊기면 `idle`로
  강등한다(아래 두 항목 모두 `follow`/`coupled`를 함께 취급하도록
  고쳤다). 실제 명령 계산은 이 노드가 아니라
  [coupled_drive](../mission/coupled_drive.md)가 한다 — arbiter는
  `cmd_vel/coupled`를 그대로 골라 재발행할 뿐이다.
- `follow`/`coupled` 진입은 `fleet_alive()`(즉 `/fleet/state`가
  `fleet_state_timeout_s` 안에 수신 중)일 때만 허용되고, 진입 후 끊기면
  스스로 `idle`로 강등한다.
- **`follow` 모드의 주행 소스는 고정이 아니다**(2026-08-31 수정,
  [TODO.md](../../TODO.md) 16번): 현재 액션이 `CARGO_ACTIONS`
  (`approach`/`turn`/`load`/`hold`/`couple`/`release`, `turn`/`couple`은
  v1.2에서 추가 — [TODO.md](../../TODO.md) 18번)면 `cmd_vel/cargo`를,
  아니면
  `cmd_vel/follow`를 쓴다 — 여름 이동 구간엔 `follow`인 채로 1호기가
  화물 명령을 보내는 게 자연스러워서다. 예전엔 여기서 무조건 `follow`를
  써서, `mission/action`은 정상 갱신되는데 실제 모터는 안 움직이는
  버그가 있었다(`~/ros2_ws`와의 비교로 발견).
- `rc`는 `follow`와 달리 진입 게이트가 없다 — `manual`과 동일하게 취급하며,
  신호가 없으면 아래 `source_timeout_s` 일반 안전장치가 그대로 0을 낸다.
  링크 상태는 `rc_alive()`로 판정해 `drive/status.rc_link`로만 보여준다
  (모드 진입을 막지는 않음). 토픽 이름·채널 매핑은 임시값 —
  [rc_bridge 문서](../fleet/rc_bridge.md), [TODO.md](../../TODO.md) 14번.
- **소스 선택 우선순위**(`_source()`): `coupling`(진행 중이거나 error)이
  모드보다 항상 위 → 모드가 `coupled`면 그대로 `coupled`(2026-09-01
  추가 — 결합 이동은 미션 명령과 무관하게 1호기 속도만 복제) → `manual`
  → `rc` → `follow`/`auto`일 때 각각 액션에 따라 `cargo`/`follow`.
- **안전 우선순위**: `emergency_stop` > `safety_stop` > 소스 timeout
  (`source_timeout_s`) > 선택된 소스의 명령.
- `mission/action`은 **바뀔 때만** 발행한다 — 같은 명령을 반복하면
  `cargo_load`의 단계 전이가 되감기기 때문(hold 재수신 시 locked→loaded).

## 연결

- 위: `drive/manual`/`ui/console`(`cmd_vel/manual`), `mission/follow_leader`
  (`cmd_vel/follow`), `mission/cargo_load`(`cmd_vel/cargo`),
  `mission/coupling`(`cmd_vel/coupling`),
  `mission/coupled_drive`(`cmd_vel/coupled`, 2026-09-01 추가),
  `fleet/rc_bridge`(`cmd_vel/rc`), `fleet/follower`(`fleet/action`),
  `ui/console`(`mission/request`, `drive/mode_cmd`).
- 아래: `base/motor`(`cmd_vel` 유일 구독), 모든 미션 노드(`mission/action`).

## 구현 상태

완성. 다만 `rc` 모드/소스 선택 로직 자체는 자동화 테스트
(`src/drive/test/test_arbiter_rc_mode.py`)로만 검증됐고, 실제 RC 신호로
움직이는 것까지는 [rc_bridge](../fleet/rc_bridge.md) 쪽 미확정 사항이
풀려야 확인 가능하다.

## 알려진 이슈

없음(모드 선택 로직 자체는) — `rc` 모드의 실측 여부는
[TODO.md](../../TODO.md) 14번 참고.

## 결합 완료 시 모드 자동 승격 (2026-09-03 추가)

`mission/coupling/status`의 `phase`가 `locked`가 되면 arbiter가 스스로
모드를 `coupled`로 올린다. 대회는 수동/자동 주행 구간이 나뉘어 있고 자동
구간에서는 사람이 `drive/mode_cmd`를 쏠 수 없는데, 예전에는 결합이 끝나도
모드가 그대로여서 협조 주행이 시작되지 않고 멈춰 섰다(TODO 22번).

`manual`/`rc`에서는 올리지 않는다 — 사람이 쥔 조종을 뺏지 않기 위해서다.
Fleet Protocol은 고정 계약이라 건드리지 않았다(`ACTIONS`에 `coupled`를
추가하지 않고 2호기 안에서 닫았다).
