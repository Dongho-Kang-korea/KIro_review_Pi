# coupled_drive — `src/mission/mission/common/coupled_drive.py`

**최신화**: 2026-09-04 (입력을 `/fleet/cmd_vel/unit2` 로 교체, 강체 조향 복제 켬 — 신규 2026-09-01)

## 역할

결합이 `locked`된 뒤 1호기와 같은 속도로 굴러가는 협조 주행을 낸다.
계절과 무관하게 항상 뜬다.

**왜 추종(`follow_leader`)을 재사용하지 않는가**: 추종은 마커 거리를
`target_distance_mm`로 맞추는 거리 제어다. 결합 중에는 결합 로드가
거리를 물리적으로 고정하므로, 거리 제어기가 이미 고정된 값을 맞추려
들며 로드와 싸운다. 그래서 결합 이동은 거리 제어가 아니라 **속도
복제**여야 한다.

**왜 그냥 끌려가지 않는가**: 주행축은 초기화 때 closed-loop로 올라가
속도 0을 유지한다 — 명령을 안 주면 바퀴가 잠겨 1호기가 2호기를
끌어야 한다. 눈길·경사가 있는 제설 구간에서는 2호기도 자기 몫을
굴려야 한다(`coupling.py`의 `verify_pull`이 2호기 모터로 당겨 전류를
보는 것도 같은 전제).

## 실행

`mars_launch/robot.launch.py`, `test_drive.launch.py`에서 뜬다.

## 설정

`src/mission/config/coupled.yaml`

| 키 | 뜻 |
|---|---|
| `enabled` | 노드 자체를 끄는 스위치 |
| `leader_timeout_s` | 1호기 속도가 이 시간 끊기면 0을 낸다. 1호기가 50 Hz로 보내므로 0.5초는 25프레임을 놓친 뒤 — 지터가 아니라 확실한 두절이다 |
| `require_locked` | 결합이 `locked`일 때만 굴린다. 결합 없이 부호만 확인할 때 false |
| `linear_sign` | ★ **실측 대상.** 2호기가 적재 과정에서 180도 돌아 붙는 구조라 1호기의 전진이 2호기에서는 후진일 수 있다 |
| `follow_angular` | 조향까지 복제할지. **강체 결합이라 켜 둔다** — 아래 참고 |
| `angular_sign` | ★ **실측 대상.** 180도 돌아 붙으면 회전 부호도 같이 뒤집힌다 |
| `max_linear_mps` / `max_angular_rps` | 2호기 자체 안전 상한. 1호기도 자기 쪽에서 이미 조여서 보낸다 |
| `publish_rate_hz` | 1호기 발행(50 Hz)과 맞춘다. 느리면 그만큼 2호기 바퀴가 늦게 따라간다 |

숫자 값은 yaml이 최신 출처다 — 여기 옮겨 적지 않는다.

### `follow_angular` 를 켠 이유 (2026-09-04, 결합부 강체 확인)

강체로 붙으면 두 호기는 하나의 긴 물체이고, **강체의 각속도는 어느 지점에서나
같다.** 그래서 2호기 궤도도 1호기와 같은 각속도로 명령해야 한다. 좌우를 같은
속도로 굴리면(조향 0) 2호기는 회전을 막는 쪽으로 버텨서 1호기가 그 저항까지
이겨야 한다 — 잠긴 캐스터와 같다.

**힌지형으로 바뀌면 반드시 다시 꺼야 한다.** 조인트 각을 모르는 채로 조향을
복제하면 잭나이프가 난다. 그 경로는 코드에 그대로 남아 있다.

## 토픽

- 구독:
  - `/fleet/cmd_vel/unit2` (Twist, 절대경로, 호기 간 규약) — 1호기
    `drive/coupled.py`가 자기 구동 명령과 같은 루프에서 50 Hz로 낸다.
    **QoS를 1호기와 똑같이 맞춰야 한다**(`BEST_EFFORT`/`KEEP_LAST` depth 1/
    `VOLATILE`). 하나라도 다르면 ROS 2는 연결을 안 맺고 **아무 경고도 안
    낸다** — 조용히 안 움직이는 것이 이 배선의 유일한 실패 모드다.
  - `mission/coupling/status` (String JSON — `phase`만 봄)
- 발행: `cmd_vel/coupled` (Twist, arbiter의 SOURCES 중 하나),
  `mission/coupled/status` (String JSON — `active`/`reason`/`coupling_phase`/
  `leader_linear`/`leader_angular`/`leader_fresh`/`linear`/`angular`)

## 동작 요약

- `publish_rate_hz`로 `_tick()`이 돌며, 굴리면 안 되는 이유(`_blocked()`)가
  있으면 정지(0,0)를 낸다: `disabled`(비활성화), `not_locked`
  (`require_locked`인데 결합 phase가 `locked`가 아님),
  `leader_velocity_missing`(1호기 속도를 아직 한 번도 못 받음),
  `leader_velocity_stale`(`leader_timeout_s` 안에 갱신 안 됨). 이유가 없으면
  `coupled_command()`(순수 함수, `linear_sign`/`angular_sign` 부호 보정 +
  상한 clamp)로 계산한 속도를 낸다.
- Twist는 값이 항상 들어 있으므로 v1.0의 "왔지만 값이 없다" 구분은 사라졌다.
  1호기는 막혔을 때도 0을 계속 보내므로, `stale`은 곧 **통신 두절**을 뜻한다.

## 연결

- 위: 1호기 `drive/coupled.py`(`/fleet/cmd_vel/unit2`) —
  [COMMUNICATION_PROTOCOL_UNIT2.md](../../COMMUNICATION_PROTOCOL_UNIT2.md)
  "결합 이동" 절 참고. `mission/coupling`(`mission/coupling/status`의 `phase`).
- 아래: `drive/arbiter`가 `cmd_vel/coupled`를 구독하며, 모드가 `coupled`일 때
  (모드보다 우선하는 `coupling` 진행 중이 아닌 한) 이 소스를 채택한다 —
  [arbiter](../drive/arbiter.md) 참고. 2호기가 `coupled` 모드로 전환되는 것
  자체는 이 노드가 아니라 운용자/1호기가 `drive/mode_cmd`로 한다.

## 구현 상태

**코드 완성·단위 테스트만 통과. 실물 미검증.**
`src/mission/test/test_coupled_drive.py`가 `coupled_command()`(부호 보정·
클램프)를 순수 함수로 검증한다. ROS 배선(토픽·QoS·타임아웃 판정)과 실물
동작은 아직 확인 전이다.

실기에서 확인할 순서:

1. `ros2 topic hz /fleet/cmd_vel/unit2` — 50 Hz로 잡히는지. 안 잡히면 QoS나
   `ROS_DOMAIN_ID`부터 본다. 여기서 막히면 그 아래는 볼 필요가 없다.
2. `linear_sign` / `angular_sign` — **반드시 낮은 속도로.** 부호가 반대면
   두 호기가 정면으로 서로를 민다.
3. `coupled` 모드에서 `drive/arbiter`가 `cmd_vel/coupled`를 실제로 고르는지.

## 알려진 이슈

- 1호기 `/mode`를 `coupled`로 바꾸는 자동 전환이 없다 — 웹 콘솔에서 사람이
  눌러야 한다. 2호기는 결합 완료 시 자동 승격하는데 1호기는 아니라, 이
  비대칭이 의도된 것인지 확인이 필요하다.
- `linear_sign`/`angular_sign` 실측과 실물 결합 상태 전체 동작 확인은
  [TODO.md](../../TODO.md) 19번에서 진행 상황을 본다.
