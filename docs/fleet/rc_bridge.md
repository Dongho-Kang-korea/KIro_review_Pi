# rc_bridge — `src/fleet/fleet/rc_bridge.py`

**최신화**: 2026-09-01

## 역할

1호기에만 물려 있는 RC(무선 조종기) 수신기의 채널 값을 받아 이 호기를
조종한다. 1호기가 `/receiver/channels`(`std_msgs/UInt16MultiArray`,
14채널, 각 1000~2000·중립 1500)를 그대로 방송하므로, 이 노드가 CH7(호기
선택 스위치)로 "지금 이 채널이 2호기 몫인지"를 스스로 걸러낸다 — 별도의
호기별 중계 토픽은 없다.

**⚠️ 채널 배치·임계값은 2026-08-30 1호기 쪽과 합의한 값이지 실물로 확인된
값은 아니다.** 배선 후 `rc/status`로 raw 판정 결과(선택 여부/정지 여부 등)
가 기대한 대로 오는지 먼저 확인할 것 — [TODO.md](../../TODO.md) 14번.

## 실행

`mars_launch/robot.launch.py`에서 `unit` 인자와 함께 상시 실행. 벤치 시험용
`web_manual.launch.py`에도 `start_fleet` 인자에 묶여 같이 뜬다(1호기가 없는
벤치에서는 채널 데이터가 안 오니 웹 콘솔의 "조종 방식" 토글/모드 버튼
노출만 미리 확인 가능).

## 설정

`src/fleet/config/rc_bridge.yaml` — 채널 인덱스·임계값 전체 목록은 yaml
안 주석 참고. 핵심만:

- `topic` — 비우면 `fleet/protocol.py`의 `TOPIC_RECEIVER_CHANNELS`
  (`/receiver/channels`)를 쓴다. 1호기 쪽 이름이 바뀌면 이 값만 바꾸면
  된다.
- `steering_channel`/`throttle_channel`(CH1/CH2), `solenoid_channel`(CH3),
  `speed_scale_channel`(CH5/VRA), `robot_select_channel`(CH7),
  `stop_channel`(CH8), `conveyor_channel`(CH9), `mode_channel`(CH10) —
  전부 1-indexed.
- `max_linear`/`max_angular`(기본 0.30/0.50, `drive/config/arbiter.yaml`의
  `/**/manual` 상한과 동일하게 시작 — VRA 스케일이 곱해지므로 이게
  최고 속도).
- `signal_timeout_s`(기본 1.0) — 솔레노이드 안전 복귀 판정 전용(아래
  참고). 주행/컨베이어는 각자 자체 timeout이 있어 이 값과 무관.

## 채널 배치 (2026-08-30 1호기·2호기 합의)

| CH | 기능 | 2호기 처리 |
|---|---|---|
| 1 | 주행 좌우(angular) | steering, 데드밴드 정규화 → `cmd_vel/rc.angular.z` |
| 2 | 주행 전후(linear) | throttle, 데드밴드 정규화 → `cmd_vel/rc.linear.x` |
| 3 | (1호기 카메라 틸트) | **솔레노이드로 재사용** — 중앙값(1500) 임계, 스틱 내림=잠금/올림=해제(추정) |
| 4 | 미사용 | 무시 |
| 5 (VRA) | 속도 스케일 노브 | 0.0~1.0 선형(1호기 `knob_scale()`과 동일), steering/throttle에 곱함 |
| 6 (VRB) | 폐기 | 무시(1호기도 폐기 — CH9가 고정속 스위치가 되며 무의미) |
| 7 | 호기 선택 스위치 | ≥1650 → "2호기". 그 미만(중간 구간 포함)은 전부 "2호기 아님" |
| 8 | 정지/재생 게이트 | ≥1650 → "재생". 그 미만은 정지 — **이동만** 0, 솔레노이드는 안 건드림 |
| 9 | 컨베이어 3단 스위치 | <1250 CCW(-1.0) / >1750 CW(+1.0) / 그 사이 정지 → `roller_manual_cmd` |
| 10 | 자동/수동 | ≥1500 → 2호기를 `auto`로, 미만 → `rc`로 원격 전환(`drive/mode_cmd`) |

## 토픽

- 구독:
  - `topic`(절대경로, 1호기 발행) — `UInt16MultiArray`, 14채널.
  - `rc/enabled`(Bool) — 웹 콘솔의 "조종 방식" 토글. `POST
    /api/unit/<n>/rc/enabled`가 이 토픽에 발행한다.
- 발행:
  - `cmd_vel/rc`(Twist) — arbiter가 `rc` 모드일 때 채택.
  - `roller_manual_cmd`(Float32, -1~1) — `cargo_load.py`가 idle/stop일
    때만 채택(웹 콘솔 컨베이어 슬라이더와 같은 토픽을 공유).
  - `solenoid_cmd`(Bool, true=잠금) — `coupling.py`/웹 콘솔과 같은 토픽을
    공유(값이 바뀔 때만 발행).
  - `drive/mode_cmd`(String) — CH7이 2호기를 선택 중일 때만, CH10에 따라
    `auto`/`rc` 중 바뀐 값만 발행.
  - `rc/status`(String JSON, 5Hz) — `enabled`/`connected`/`age_s`/
    `robot_selected`/`run_active`/`requested_mode`/`solenoid_locked`/
    `linear`/`angular`/`roller`. 웹 콘솔이 그대로 반영해 보여준다.

## 동작 요약

### "CH7 이탈 시 유지" — 정확히 무슨 뜻인가

CH7이 1호기 쪽으로 가 있어도(또는 애매한 중간 구간이어도) 2호기는 **직전에
받은 조향/스로틀/컨베이어/모드를 그대로 계속 낸다** — 조종 대상이 바뀌었다고
2호기가 갑자기 멈추지 않는다. 구현은 단순하다: `_on_channels`가 매 프레임
`self.last_twist`/`self.last_roller`를 무조건 재발행하고, CH7이 2호기를
가리킬 때만 그 값들을 새로 계산해 갱신한다 — 그 외에는 그냥 마지막 값을
반복 재발행할 뿐이다.

**0으로 완전히 끊기는 경우는 딱 둘뿐이다**:

1. **`/receiver/channels` 토픽 자체가 끊김**(수신기 연결 끊김) — 이때는
   `_on_channels`가 아예 안 불리니 재발행도 자동으로 멈춘다. `cmd_vel/rc`는
   arbiter의 `source_timeout_s`가, `roller_manual_cmd`는 `cargo_load`의
   수동 롤러 timeout이 각자 알아서 0으로 내린다 — 이 노드가 따로 손 안 대도
   된다. 유일한 예외는 솔레노이드: 자체 timeout이 없는 단순 래치라
   (`base/solenoid.py`), `signal_timeout_s`(기본 1초) 안에 새 프레임이
   없으면 이 노드가 직접 `solenoid_cmd=true`(잠금)를 강제 발행한다.
2. **웹 콘솔 "조종 방식" 토글을 RC→웹으로 끔** — 이건 CH7 이탈과 다르게
   **즉시** 주행/컨베이어를 0으로 만든다(사람이 명시적으로 통제권을 가져간
   것이므로 "유지"하지 않는다). 솔레노이드는 이 경우에도 안 건드린다 —
   커플링 상태를 토글 하나로 바꾸는 건 원치 않는 부작용이 될 수 있어서다.

### CH8('정지') 의 범위

`safety/stop`이나 `safety/emergency_stop`처럼 이미 다른 노드
(`fleet/follower.py`)가 발행하는 공유 안전 토픽에는 손대지 않는다 — 여러
발행자가 같은 Bool 토픽에 쓰면 누가 마지막에 썼는지로 결정되는 경쟁 상태가
생기기 때문이다. 대신 CH8='정지'는 **이 노드 안에서만** 게이트로 쓴다 —
`cmd_vel/rc`/`roller_manual_cmd`를 0으로 유지하되, 그 외 아무 공유 토픽도
건드리지 않는다. 진짜 비상정지가 필요하면 물리 estop 버튼(웹 콘솔)을 쓴다.

### CH10('자동/수동') 원격 모드 전환과 웹 잠금

CH7이 2호기를 선택 중일 때만, CH10 값에 따라 이 호기의 `drive/mode_cmd`를
직접 발행한다(`auto` 또는 `rc`). 웹 콘솔의 "모드" 버튼과 같은 토픽에 쓰므로,
두 쪽이 동시에 활성화돼 있으면 경쟁이 생긴다 — 그래서 `rc/enabled`가
켜져 있는 동안(`rc_owns()`)은 웹 콘솔 쪽 `api_mode`가 **서버에서 409로
거부**한다(2026-09-01부터, TODO.md 21번). `unit.html`의 모드 버튼 잠금은
그걸 미리 보여주는 UI일 뿐이고, 실제 중재는 이제 코드 레벨이다 — 모드뿐
아니라 주행/솔레노이드/컨베이어까지 같은 기준으로 막힌다([console
문서](../ui/console.md) 참고).

## 연결

- 위: 1호기 RC 채널 방송(`/receiver/channels`), 웹 콘솔(`rc/enabled`).
- 아래: `drive/arbiter`(`cmd_vel/rc`, `rc`/`auto` 모드 전환),
  `mission/cargo_load`(`roller_manual_cmd`), `base/solenoid`
  (`solenoid_cmd`), `ui/console`(`rc/status` 구독해 표시).

## 구현 상태

**코드·자동화 테스트는 완성, 실물 검증은 전혀 안 됨.** 순수 로직(정규화,
knob_scale, 컨베이어 3단, 솔레노이드 방향, CH7/CH8/CH10 게이팅, "유지"
동작, 웹 토글의 즉시 0, 워치독의 솔레노이드 안전 복귀)은
`src/fleet/test/test_rc_bridge.py`로 자동화 테스트했다(2026-08-30, 20개
통과). "실제 1호기 채널 값이 이 표대로 오는가"는 배선 후 실물 확인 대상.

## 알려진 이슈

- 채널 배치·임계값·방향 전부 미검증 — [TODO.md](../../TODO.md) 14번 참고.
- CH7 선택 밴드는 1/2호기 2단 스위치 기준이다. 3호기는 설계에서
  빠졌으므로(TODO 19) 밴드를 더 나눌 일은 없다.
- `solenoid_cmd`/`roller_manual_cmd`/`drive/mode_cmd` 전부 다른 발행자
  (coupling/console)와 같은 토픽을 공유한다 — "동작 요약"의 경쟁 상태
  설명 참고, 새로 발행자를 추가할 때는 이 구조를 먼저 이해할 것. console
  쪽은 2026-09-01부터 `rc_owns()`가 켜져 있는 동안 자기 쪽 발행(주행/
  솔레노이드/컨베이어/모드)을 서버에서 막으므로 실질적인 경쟁은 사라졌다
  (TODO.md 21번) — 다만 이건 console이 협조적으로 양보하는 구조이지,
  코드 레벨의 진짜 우선순위 중재(예: 둘 다 값을 내도 한쪽이 항상 이기는
  방식)는 여전히 아니다.
