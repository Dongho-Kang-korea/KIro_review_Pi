# imu_aruco_360_test — `src/base/base/imu_aruco_360_test.py`

**최신화**: 2026-09-03 (신규 문서 — 2026-09-01 `mission/common`에서 이 패키지로 이동)

## 역할

IMU(`heading_offset_deg`) 영점 오차를 ArUco 마커를 기준점 삼아 자동으로
재는 **일회성 검증 도구**다([TODO.md](../../TODO.md) 3번). "알려진 방위로
정렬 후 육안 비교"하던 예전 방식 대신, 로봇을 360도 회전시키며 IMU
누적 각도와 마커 재검출 오차를 비교해 실측 오차를 산출한다.

`armed` 파라미터가 `true`로 명시되지 않으면 0속도 명령만 내는 완전
비활성 상태다 — 실수로 도는 것을 막는 안전장치. `arbiter`의 `manual`
모드(`cmd_vel/manual`)로 명령을 내므로 안전 정지·명령 타임아웃 등
평소 주행 경로를 그대로 탄다.

**동작 순서**: (1) arbiter를 idle로 두고 서로 다른 유효 ArUco pose
프레임을 모음 → (2) manual로 전환해 IMU 누적 각도를 보며 반시계로
1회전 → (3) 누적 각도가 360도에 도달하면 정지 → (4) 같은 마커를
재검출해 (1)단계 대비 yaw 오차를 측정 → (5) 그 오차만큼 마커 yaw가
(1)단계 값과 맞을 때까지 추가 회전하며, 이때의 추가 IMU 회전량도
별도로 누적·보고한다.

## 실행

어떤 launch에도 포함되지 않는다(`calibrate_camera`와 같은 패턴). 수동으로:

```bash
ros2 run base imu_aruco_360_test --ros-args -r __ns:=/unit2 -p armed:=true
```

`base/marker_vision`(마커), `base/imu`, `drive/arbiter`가 먼저 떠 있어야
한다(`robot.launch.py`/`test_drive.launch.py`로 이미 뜬 상태에서 이
노드만 추가로 띄운다).

## 설정 (런치 파일 없음 — `ros2 run` 인자로 넘긴다)

`armed`(기본 false, 위 "역할" 참고), `marker_id`(기본 1), 마커 표본 수
(`initial_marker_samples`/`post_turn_marker_samples`), 각 단계 타임아웃
(`sensor_timeout_s`/`prepare_timeout_s`/`mode_timeout_s`/
`full_turn_timeout_s`/`reacquire_timeout_s`/`align_timeout_s`), 1회전
구간 속도·게인(`full_turn_target_deg`/`full_turn_max_rps`/
`full_turn_min_rps`/`full_turn_kp_rps_per_deg`/`wrong_direction_limit_deg`
/`max_imu_step_deg`/`settle_s`), 재정렬 구간(`marker_yaw_tolerance_deg`/
`align_kp_rps_per_deg`/`align_min_rps`/`align_max_rps`/`align_hold_s`/
`align_hold_frames`), `publish_rate_hz`(기본 20.0).

## 토픽

- 구독: `imu/heading_deg` (Float32), `marker/status` (String JSON),
  `drive/status` (String — 현재 모드 확인), `safety/stop`/
  `safety/emergency_stop` (Bool)
- 발행: `cmd_vel/manual` (Twist — arbiter의 `manual` 소스로 명령),
  `drive/mode_cmd` (String — idle↔manual 전환), `test/imu_aruco_360/status`
  (String JSON, TRANSIENT_LOCAL — 마지막 결과를 늦게 구독해도 받음)

## 연결

- 위: `base/imu`(`imu/heading_deg`), `base/marker_vision`(`marker/status`),
  `drive/arbiter`(`drive/status`, `cmd_vel/manual`/`drive/mode_cmd`를
  통해 간접 제어).
- 아래: 없음(결과는 `test/imu_aruco_360/status`로 사람이 확인 후
  `imu.yaml`의 `heading_offset_deg`에 수동 반영).

## 구현 상태

**완성, 실측 도구로 5회 실행 완료**(2026-09-01, [TODO.md](../../TODO.md)
3번). `heading_offset_deg`는 아직 `0.0` 그대로라, 이 도구로 잰 오차값을
실제로 오프셋에 반영할지·반영한다면 얼마로 할지는 아직 결론이 안
났다 — [imu](imu.md), [TODO.md](../../TODO.md) 3번 참고.

## 알려진 이슈

없음 — 발견되면 [TODO.md](../../TODO.md)에 등록한다.
