# motor — `src/base/base/motor.py`

**최신화**: 2026-09-03

## 역할

`cmd_vel`(Twist)을 받아 좌/우 구동 모터 + 독립 롤러 1개의 ODrive/Steadywin
CANSimple 프레임으로 바꾼다. `test_single_id`가 양수면 단일 모터만 구동하는
시험 모드로 전환된다. 주행 모드가 몇 개든 이 노드는 모른다 — 소스를 하나로
고르는 일은 `drive/arbiter`가 한다.

## 실행

`mars_launch/robot.launch.py`, `manual.launch.py`, `manual_drive.launch.py`,
`test_drive.launch.py`(robot.launch.py를 include), `web_manual.launch.py`
전부에서 뜬다 — 모터 없이 뜨는 launch는 없다.

## 설정

`src/base/config/motor.yaml` — CAN 채널/비트레이트, 모터 ID(좌/우/롤러),
가감속 상한(`max_accel`/`max_decel`), `cmd_vel_timeout_s`/
`roller_cmd_timeout_s`, `vel_scale`/`roller_scale`, `contact_current_threshold_a`,
`can_reconnect_interval_s`(기본 2초, 2026-09-01 추가 — 어댑터 재연결
재시도 간격) 등. 숫자는 이 파일이 최신 출처다.

## 토픽

- 구독: `cmd_vel` (Twist, 주행), `roller_cmd` (Float32 -1.0~1.0, 롤러 독립)
- 발행: `motor/status` (String JSON — 모터별 target/applied 속도, forward_revs,
  axis_error/axis_state, current_a/iq_setpoint(RTR 활성 시 — 기본 활성),
  can_connected,
  contact_current_threshold_a, current_seq)

## 동작 요약

- **zero-first 초기화**: 시작 시 주행축에 0속도부터 보낸 뒤 closed-loop 진입.
  롤러는 idle(토크 해제) 상태로 시작.
- **watchdog**: `cmd_vel`이 `cmd_vel_timeout_s`(기본 0.5s) 안 오면 주행축만
  0으로 램프다운. `roller_cmd`가 `roller_cmd_timeout_s` 안 오면 롤러 토크를
  해제한다(`_release_roller`) — `cargo_load`가 적재 도중 죽어도 롤러만 계속
  도는 상태를 막는다.
- **가감속 램프**: `max_accel`/`max_decel`로 매 tick 접근시킨다. 반대 방향
  명령은 일단 0까지 감속 후 재가속.
- **CAN 전송 스로틀**: `send_velocity_if_changed`가 `send_threshold` 이상
  변화했거나 `keepalive_interval`이 지났을 때만 전송 — 매 tick(50Hz) 전송하면
  USB CAN 송신큐(ENOBUFS)가 넘친다.
- **CAN 실패 backoff**: 송신 실패 시 1초간 재시도 중단, 5초마다만 경고 로그.
- RTR 폴링(`enable_rtr_requests`)은 **기본 활성**이다(2026-08-31 실측 후 전환).
  컨트롤러는 `0x09`(엔코더)를 100Hz로 알아서 뿌리지만 **`0x14`(전류 Iq)는
  뿌리지 않는다** — RTR로 물어봐야만 답한다. 끄면 `current_a`가 영영 `null`
  이라 아래 접촉 판정이 통째로 죽는다. 실측에서 `0x14`·`0x17`(온도)은 응답,
  `0x18`(버스 전압)은 무응답이었다.
- **CAN 어댑터 자동 재연결(2026-09-01 추가)**: PCAN-USB가 접촉 불량으로
  탈부착되면(`dmesg`: `Rx urb aborted`, `can0 removed`) 소켓이 죽어
  `[Errno 19] No such device`가 난다. 예전에는 이 예외가 노드까지
  올라가 프로세스가 죽어서 어댑터가 다시 붙어도 모터 제어가 영영
  안 돌아왔다(실측 확인). 지금은 실패한 버스를 놓고(`_drop_bus`)
  `can_reconnect_interval_s`(기본 2초)마다 재오픈을 시도하며
  (`_open_bus`), 성공하면 모터를 재초기화한다 — 노드는 죽지 않는다.
  재연결 대기 중에는 `motor/status`의 `can_connected`가 `false`로
  나간다.

## 연결

- 위: `drive/arbiter`가 유일하게 `cmd_vel`을 발행하도록 설계됨(다른 발행자가
  동시에 쓰면 충돌). `roller_cmd`는 `mission/cargo_load`가 발행.
- 아래: 없음 (하드웨어 최하단, CAN 버스 직접 제어).
- `mission/coupling`과 `mission/cargo_load`는 `motor/status`의 `current_a`를
  구독해 접촉 판정에 쓴다. `motor.yaml`의 `contact_current_threshold_a`는
  이제 **공통 폴백**이다 — 각 미션 config(`coupling.yaml`/`cargo.yaml`)에
  같은 이름의 키가 있고 그쪽이 우선한다(2026-09-01, 접근 속도가 달라
  접촉 전류가 다르기 때문). 모두 0.0이면 두 미션
  모두 접촉 판정을 시작하지 않는다(`base/config/motor.yaml`에서 공유).

## 구현 상태

완성. `require_can: false`(기본)이면 CAN 버스가 없어도 "안전 미연결 모드"로
기동한다.

## 알려진 이슈

**2026-08-28**: 웹 콘솔 수동 컨베이어 조작을 디버깅하다가 발견 —
`roller_cmd`가 도착하면 `roller_torque_enabled: true`, `target`/`applied`
값(예: 2.5)까지 정상 반영되고 CAN 자체는 `can_connected: true`인데,
`motors.3`(롤러, `roller_id`)의 `axis_error`/`axis_state`/
`encoder_velocity`/`current_a`가 **전부 `null`** — 좌/우 구동 모터
(`motors.1`/`.2`)는 이 값들이 정상 채워지는 것과 대조적이다. 소프트웨어
체인(브라우저→콘솔→`cargo_load`→`roller_cmd`→`motor.py`→CAN 프레임
전송)은 끝까지 정상 동작을 확인했다 — 롤러 컨트롤러가 CAN 하트비트에
응답을 안 하고 있는 것으로 보인다(전원/배선 쪽 원인 추정).
[TODO.md](../../TODO.md)에 등록.
