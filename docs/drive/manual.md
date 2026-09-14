# manual — `src/drive/drive/manual.py`

**최신화**: 2026-08-23

## 역할

터미널 키보드 텔레오퍼레이션(WASD)으로 `cmd_vel/manual`을 발행하는 시험용
입력 노드. 최종 `cmd_vel`은 `drive/arbiter`가 선택하며, arbiter 모드가
`manual`이어야 이 후보가 채택된다.

## 실행

**launch에 포함되지 않는다** — 실제 터미널(TTY)이 필요해서다. 별도 창에서
직접 실행:

```
export ROS_DOMAIN_ID=10
ros2 run drive manual --ros-args -r __ns:=/unit2
```

`manual_drive.launch.py`는 `start_manual:=true`(기본)일 때 이 노드를 포함해
격리된 test_domain(기본 77)에서 실행할 수 있다 — 이 경우는 emulate_tty로
launch가 TTY를 확보한다.

## 설정

`src/drive/config/arbiter.yaml`의 `manual` 구역 — `linear_step`,
`angular_step`, `max_linear`(기본 0.30), `max_angular`(기본 0.50),
`repeat_rate_hz`(기본 20). `ros2 run`으로 띄우면 yaml이 실리지 않으므로
`declare_parameter` 기본값이 쓰인다 — 바꾸려면
`--ros-args -p max_linear:=0.2`로 덮어쓴다.

## 토픽

- 발행: `cmd_vel/manual` (Twist)

## 동작 요약

- w/s: 전진/후진, a/d: 좌/우 회전, space: 즉시 정지, q 또는 Ctrl-C: 종료.
- `repeat_rate_hz`로 **키를 안 눌러도 계속 재발행**한다 — arbiter의
  `source_timeout_s`(기본 0.5s)에 걸려 끊기지 않게 하기 위함.
- 이 노드가 죽으면 발행이 멈추고, arbiter가 timeout으로 스스로 0을 낸다 —
  그것이 이 경로의 안전장치다.
- `stdin`이 TTY가 아니면(즉 launch로 잘못 띄우면) 에러 로그만 남기고
  종료한다.

## 연결

- 위: 사람(키보드 입력).
- 아래: `drive/arbiter`(`cmd_vel/manual` 후보로 구독, 모드가 `manual`일
  때만 채택).

## 구현 상태

완성.

## 알려진 이슈

없음 — 발견되면 [TODO.md](../../TODO.md)에 등록한다.
