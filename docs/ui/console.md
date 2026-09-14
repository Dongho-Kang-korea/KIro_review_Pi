# console — `src/mars_console/mars_console/console.py`

**최신화**: 2026-09-01

## 역할

MARS 운용 콘솔. **로봇이 아니라 운용 PC에서 띄운다.** 1·2호기 전체를
한 화면에서 관리한다 — 호기 선택, 모드 전환, 수동 조작, 비상 정지, 여름
미션 명령. 1호기와 2호기는 토픽 네임스페이스와 모드를 거는 방식이 달라서
(`UNITS` 표가 그 차이를 흡수), 이 노드가 호기별 접두사/모드 프로토콜을
UI 뒤에서 대신 처리한다.

## 실행

로봇 SBC가 아니라 운용 PC에서:

```
export ROS_DOMAIN_ID=10
ros2 run mars_console console            # http://localhost:5100
```

`web_manual.launch.py`에도 포함돼 있다(2호기 **단독 벤치 시험** 환경 일괄
실행 — `start_camera`(기본 true)로 camera+marker_vision, `start_fleet`
(기본 true)로 fleet/follower+mission/follow_leader(추종 모드 시험용)도
같이 뜬다. 필요 없으면 각각 `:=false`). `base/solenoid`,
`mission/cargo_load`는 게이트 없이 항상 같이 뜬다 — 아래 솔레노이드/
컨베이어 수동 조작이 이 두 노드를 전제로 한다.

**2026-08-29부터 `start_fake_leader`(기본 true)로 [fleet/fake_leader]
(../fleet/fake_leader.md)도 같이 뜬다** — 1호기 없이도 `/fleet/state`를
대신 흉내 내 웹 콘솔에서 바로 "추종" 모드를 켜볼 수 있다(거리 제어는
ArUco 마커 pose로, UWB 미장착 상태에서도 동작). 실물 1호기와 같은
`ROS_DOMAIN_ID`에서는 반드시 `start_fake_leader:=false`로 꺼야 한다.

## 설정

`src/mars_console/config/console.yaml` — `host`, `port`, `input_timeout`
(브라우저 조작 두절 시 0 발행), `max_linear`/`max_angular`(수동 주행
최대 속도 — 호기 쪽 상한과 별개로 한 번 더 깎는다, 2026-08-28 기존
실효 최대치의 2배로 상향), `stale_seconds`(온라인 판정). 숫자는 이
파일이 최신 출처다.

## 토픽

호기별로 접두사가 다르다 — 1호기는 루트, 2호기는 `/unit2`.

- 1호기: 구독 `/motor/status`, `/camera/status`, `/mission/section`,
  `/track/detections`, `/mission/trafficlight`, `/mission/escort/status`,
  `/mission/heading_status`, `/mission/marker/progress`,
  `/track/annotated/compressed`(카메라). 발행 `/mode`(중재 소스 이름
  직접 지정), `/cmd_vel_manual`. 비상 정지 토픽 없음.
- 2호기 `modes`: `manual`/`auto`/`follow`/`rc`(2026-08-30 추가 — 1호기
  경유 RC 조종기, [rc_bridge 문서](../fleet/rc_bridge.md) 참고. `follow`와
  달리 진입 게이트가 없다, `follow_available`과 짝인 `rc_available`은
  버튼을 잠그지 않고 링크 상태 표시에만 쓴다).
- 2호기: 구독 `/unit{N}/drive/status`, `/unit{N}/motor/status`,
  `/unit{N}/mission/cargo/status`, `/unit{N}/mission/coupling/status`,
  `/unit{N}/mission/follow/status`, `/unit{N}/marker/status`,
  `/unit{N}/imu/status`, `/unit{N}/uwb/status`, `/fleet/status/unit{N}`
  (절대경로), 카메라는 `/unit{N}/vision/image/compressed`(ArUco 박스
  주석, 기본)와 `/unit{N}/camera/image/compressed`(원본, 0.75초 안
  주석 프레임이 없을 때만 폴백) 둘 다 구독. 발행
  `/unit{N}/drive/mode_cmd`(manual/auto/follow 그대로), `/unit{N}/cmd_vel/manual`,
  `/unit{N}/safety/emergency_stop`, `/unit{N}/mission/request`,
  `/unit{N}/solenoid_cmd`(Bool — 수동 잠금/해제, 2026-08-28 추가),
  `/unit{N}/roller_manual_cmd`(Float32 — 수동 컨베이어, 2026-08-28 추가),
  `/unit{N}/marker/axes_enable`(Bool — ArUco 3축 좌표계 오버레이 on/off,
  2026-08-29 추가), `/unit{N}/rc/enabled`(Bool — 웹/RC 조종 방식 토글,
  2026-08-30 추가, [rc_bridge](../fleet/rc_bridge.md)가 구독). 구독 추가:
  `/unit{N}/rc/status`(String JSON — rc_bridge 상태를 `status_topics`의
  `rc` 키로 그대로 취합).

## 동작 요약

- **모드 전환 방식이 호기마다 다르다**(`mode_style`): 1호기는 `source`
  (계절→중재 소스 매핑, `UNIT1_AUTO` 표), 2호기는 `unit`(manual/auto/
  follow를 그대로 `/unit{N}/drive/mode_cmd`로 보냄).
  - `mode_style: 'source'`이고 `mode='auto'`이면 현재 `section` 값을
    `UNIT1_AUTO`로 변환해 `/mode`에 실제로 실어 보낸다(예: escort→'escort',
    나머지→'comp'). `drive/config/drive.yaml`(1호기 쪽)의 `source_topics`와
    맞아야 한다.
- **입력 감시**: `_tick_drive`가 20Hz로 돌며, `manual` 모드인 호기에 한해
  브라우저 조작이 `input_timeout` 안 오면 0을 발행 — 창을 닫거나 네트워크가
  끊겨도 마지막 속도로 계속 가지 않는다. (계속 발행은 하므로 arbiter의
  `source_timeout_s`에는 안 걸리고, 속도만 0이 된다.)
- **속도 슬라이더**(2026-08-28 추가, 순수 프런트엔드): "수동 주행" 패널의
  속도 슬라이더(20~100%)는 방향 버튼/키보드의 기본 배율(unit.html의
  `drive()` 호출부가 최신 출처)에 곱하는 배율이다 — 서버는 여전히
  `max_linear`/`max_angular`로 최종 상한을 건다(슬라이더 100%가 정확히
  이 상한에 맞춰지도록 기본 배율도 같이 조정해뒀다). "컨베이어" 패널의
  슬라이더는 -100~100%로 `roller` 값을 -1.0~1.0에 직접 매핑한다.
- `follow` 모드 요청 시 해당 호기의 `follow_available`(=`/fleet/state`
  생존)이 꺼져 있으면 API가 409로 거부한다. `rc` 모드는 이런 게이트가
  없다 — `manual`처럼 신호 유무와 무관하게 즉시 전환된다
  (`arbiter.py`의 `rc_alive()`는 `rc_available` 표시용일 뿐).
- **웹/RC 조종 방식 토글**(`POST /api/unit/<n>/rc/enabled {enabled:bool}`,
  2호기 전용, 2026-08-30 추가, **2026-09-01 배타적 전권으로 강화 —
  TODO.md 21번**): `rc/enabled`에 그대로 발행하는 단순 토글이다(3축
  오버레이 토글과 같은 패턴) — 실제 반영 여부는 `rc_bridge`가
  `rc/status.enabled`로 되돌려주므로 `snapshot()`의 `rc_enabled`는 그
  값을 그대로 보여준다(콘솔 자체 기억이 아님).
  - **`rc_owns(unit)`이 코드 레벨 중재의 단일 기준이다**: `rc/status`
    피드가 `stale_seconds`(기본 2.0초) 안의 최신 값이고 `enabled=true`일
    때만 참이다. `snapshot()`의 `rc_owns` 필드로 그대로 노출한다
    (`rc_enabled`와 달리 이건 신선도까지 반영 — `rc_bridge`가 죽어
    피드백이 끊기면 자동으로 거짓이 돼 웹이 다시 조종할 수 있다. 락이
    영구 고착되면 안 되므로 안전 기본값은 항상 웹).
  - `rc_owns(unit)`이 참이면 `api_mode`/`api_drive`/`api_solenoid`/
    `api_roller`가 전부 **409로 거부**한다(예전엔 "모드"만 UI에서
    disabled로 막고 서버는 안 막아서, RC가 켜져 있어도 API를 직접
    호출하면 `solenoid_cmd`/`roller_manual_cmd`에서 `rc_bridge`와 경쟁
    상태가 생길 수 있었다). `_tick_drive`도 같은 기준으로 `cmd_vel/manual`/
    `roller_manual_cmd` 재발행 자체를 건너뛴다 — API만 막으면 이미 쥐고
    있던 값이 `input_timeout`까지는 계속 나갈 수 있어서다.
  - `estop`/`rc/enabled`/`marker/axes`/`mission`은 예외로 항상 통과한다
    — 비상 정지와 조종 방식 전환 자체를 막으면 안 되고, 3축 오버레이·
    미션 버튼은 이번 강화 범위 밖이다.
  - `unit.html`은 `snapshot()`의 `rc_owns`를 그대로 반영해 모드 버튼뿐
    아니라 주행 패드/키보드, 컨베이어 슬라이더·버튼, 솔레노이드 버튼을
    전부 잠근다(서버가 이미 막으므로 UI 잠금은 미리 보여주는 역할).
    RC 전권으로 새로 들어가는 순간엔 웹이 쥐고 있던 주행/컨베이어 입력을
    프런트엔드에서 0으로 리셋해, 나중에 웹이 전권을 되찾을 때 예전 값이
    갑자기 재생되지 않게 한다.
  - 웹→RC 방향은 이전부터 완전히 막혀 있었다(`rc_bridge.py`가
    `enabled=false`면 채널 자체를 무시) — [rc_bridge
    문서](../fleet/rc_bridge.md) 참고.
- `set_estop(unit, false)`(해제)는 2호기 arbiter의 `reset_emergency`
  래치 해제도 함께 보낸다.
- **콘솔 내부 모드는 `drive/status`를 따라간다**: `set_mode()`로 직접
  바꾼 게 아니어도(예: launch가 `start_mode`로 미리 지정, 다른 클라이언트가
  `mode_cmd`를 보냄) `drive/status`에 새 `mode`가 오면 `self.mode[unit]`을
  그쪽으로 맞춘다 — 아래 "알려진 이슈" 참고.
- **솔레노이드 수동 조작**(`POST /api/unit/<n>/solenoid {lock:bool}`,
  2호기 전용): `solenoid_cmd`에 직접 발행한다. `coupling.py`가 `locking`
  단계 외엔 이 토픽을 안 건드리므로 평소엔 즉시 반영된다. `rc_owns(unit)`
  이면 409(위 "웹/RC 조종 방식 토글" 참고).
- **컨베이어 수동 조작**(`POST /api/unit/<n>/roller {value:-1..1}`,
  2호기 전용): 주행과 같은 20Hz 재발행 + timeout 패턴으로
  `roller_manual_cmd`를 낸다 — `cargo_load.py`가 `action`이 `idle`/`stop`일
  때만 채택하므로, 자동 미션이 시작되면 자동 쪽이 즉시 우선한다
  ([cargo_load 문서](../mission/cargo_load.md) 참고). `rc_owns(unit)`이면
  API가 409를 내고, `_tick_drive`도 재발행을 멈춘다(위 참고).
- **ArUco 3축 좌표계 표시**(`POST /api/unit/<n>/marker/axes {enabled:bool}`,
  2호기 전용, 2026-08-29 추가): `marker/axes_enable`에 그대로 발행하는
  단순 토글이다. 실제 켜짐/꺼짐은 `marker/status.axes_enabled`를 통해
  되돌아오므로 화면 버튼은 그 값을 따라간다(여러 탭에서 열어도 서로
  어긋나지 않음). unit.html "상태" 표에는 pose가 켜져 있을 때
  `x_mm`/`y_mm`/`z_mm`/`roll_deg`/`pitch_deg`/`yaw_deg` 수치도 그대로
  보여준다 — [marker_vision 문서](../base/marker_vision.md) 참고.

## 연결

- 위: 사람(운용자, 브라우저).
- 아래: 1호기 통신 노드(직접), `drive/arbiter`(2호기,
  `drive/mode_cmd`/`cmd_vel/manual`), `fleet/follower`(`/fleet/status/unitN`
  구독).

## 구현 상태

완성. 3호기는 설계에서 빠졌다(TODO 19) — UNITS 표에서 제외했다. 다시
실제로 온라인이면 그대로 동작한다. 모드 동기화, 솔레노이드/컨베이어/
ArUco 축 오버레이 수동 조작은 `src/mars_console/test/`로 자동화
테스트했다(2026-08-29, 6개 통과). **아직 안 된 것**: 실제
`web_manual.launch.py`를 재시작해 브라우저에서 축 토글 버튼과 pose
수치 표시, fake_leader로 풀리는 추종 버튼을 실물로 확인 —
[RUNBOOK.md](../../RUNBOOK.md) 5절. **2026-08-30 추가된 `rc` 모드
버튼과 "조종 방식" 웹/RC 토글**도 화면에 뜨는 것, `rc_available`/
`rc_enabled` 반영, 모드 버튼 잠금은 자동화 테스트만 됐고, 실제 1호기
RC 신호로 조종되는지는 [rc_bridge](../fleet/rc_bridge.md) 쪽 채널
배치가 실물로 확인돼야 검증 가능([TODO.md](../../TODO.md) 14번).
**2026-09-01 조종 방식을 배타적 전권으로 강화(TODO.md 21번)**:
`rc_owns()`와 `api_mode`/`api_drive`/`api_solenoid`/`api_roller`의 409
거부, `_tick_drive`의 재발행 중단, `unit.html`의 전체 패널 잠금을
`src/mars_console/test/test_rc_exclusive_control.py`(12개)로 자동화
테스트했다(신선도 만료 시 자동 해제, estop/rc-enable 예외 포함) —
전체 `mars_console` 스위트 28개, 저장소 전체 자동화 테스트 102개 모두
통과. **실물 검증은 아직**: 웹/RC를 동시에 조작해봐도 실제로 경쟁 상태가
안 생기는지, RC 전권 중 웹 화면에서 버튼들이 실제로 잠기는지는 다음
실물 시험 때 확인해야 한다.

## 알려진 이슈

없음(수정 완료). **2026-08-28**: `web_manual.launch.py`가 arbiter를
`start_mode: manual`로 띄우면, 로봇은 이미 수동 모드인데 콘솔 내부의
`self.mode[unit]`은 `set_mode()`를 거치기 전엔 항상 `idle`로 남아
있어서 — 화면엔 이미 활성화된 것처럼 보이는데(`drive/status`의 실제
모드를 그대로 표시하므로) `_tick_drive`는 계속 0만 내보내는 불일치가
있었다. `_on_status`가 `drive/status`를 받을 때마다 `self.mode[unit]`을
그 값으로 동기화하도록 고쳤다.

**추종 모드가 안 움직이는 문제(2026-08-28 조사, 2026-08-29 갱신)**:
콘솔/코드 문제가 아니다. ①`web_manual.launch.py`엔 처음엔
`fleet/follower`/`mission/follow_leader`가 아예 없어서(`start_fleet`
인자 추가로 해결) 추종을 시험할 노드 자체가 없었고, ②그걸 다 띄워도
**`/fleet/state`를 발행하는 쪽이 있어야** 움직인다 — arbiter의
`fleet_alive()` 판정과 `fleet/follower`의 자동 전환이 각각 의존하는
외부 입력이다. 실물 운용(도메인10, `robot.launch.py`)에서는 여전히
1호기가 `/fleet/state`·`/fleet/cmd/unit{N}`을 실제로 발행해야 한다 —
2호기 쪽 코드로 고칠 수 있는 부분이 아니다. **다만
`web_manual.launch.py`(도메인77, 벤치 시험 전용)는 2026-08-29부터
`start_fake_leader`(기본 true)가 `/fleet/state`를 대신 흉내 내므로
1호기 없이도 "추종" 버튼이 풀린다** — [fake_leader
문서](../fleet/fake_leader.md) 참고.
