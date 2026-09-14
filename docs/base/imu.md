# imu — `src/base/base/imu.py`

**최신화**: 2026-08-23

## 역할

BNO086(I2C) 방위각/자세를 발행한다. `enabled: false`일 때는 그냥 아무것도
열지 않고 `ready: false` 상태만 보고한다 — 노드 자체는 항상 뜬다.

## 실행

`mars_launch/robot.launch.py`, `web_manual.launch.py`(`start_imu:=true`가
기본)에서 뜬다.

## 설정

`src/base/config/imu.yaml` — `enabled`, `publish_rate`,
`heading_offset_deg`(영점), `i2c_address`(기본 0x4B), `reconnect_interval_s`.

## 토픽

- 발행: `imu/heading_deg` (Float32, 0~360), `imu/quaternion` (Quaternion),
  `imu/rpy_deg` (String, 사람이 읽는 로그용), `imu/status` (String JSON —
  enabled/ready/heading_deg/error/roll_deg/pitch_deg/yaw_deg)

## 동작 요약

- `enabled: true`인데 센서 오픈에 실패하면 `reconnect_interval_s`마다
  재시도한다(`_open`을 다시 호출).
- 초기 SHTP batch 경고(`RuntimeError`)는 이전 담당자 실기 기록과 동일하게
  연결 실패로 취급하지 않고 이후 Quaternion 읽기를 계속한다.
- heading은 yaw + `heading_offset_deg`를 0~360으로 정규화한 값이다.

## 연결

- 위: 없음(하드웨어 최상단 입력, 미장착 시 비활성).
- 아래: `mission/follow_leader`(사용 안 함, UWB/마커만 씀 — 참고: 이 노드는
  현재 추종 계산에는 안 쓰인다), `mission/cargo_load`(`imu/heading_deg`로
  90도 회전 판정), `fleet/follower`(상태 보고용 `heading_deg`).

## 구현 상태

**장착됨** — `base/config/imu.yaml`에서 `enabled: true`, I2C 주소
0x4B(=75)로 실측 확인된 값이 설정돼 있다(코드 기본값은 `false`이지만 이
yaml이 최신 출처). `heading_offset_deg`는 아직 0.0(영점 미교정).

## 알려진 이슈

`heading_offset_deg` 영점 교정이 아직이면 [TODO.md](../../TODO.md)에서
관련 항목을 확인한다. 오차 실측은 [imu_aruco_360_test](imu_aruco_360_test.md)
(일회성 검증 도구, 2026-09-01까지 5회 실행 완료)로 하되, 오프셋 반영
여부는 아직 결론이 안 났다.
