# MARS 2호기 통신 규약 v1.1

기준일: 2026-09-04 (v1.0 2026-08-05)  
적용 대상: MARS 1호기와 2호기, 2호기 내부 ROS 2 노드  
구현 기준: `fleet/protocol.py`, `fleet/follower.py`

## 1. 목적

이 문서는 1호기와 2호기 사이의 명령 및 상태 교환과 2호기 내부 노드 간 토픽을 정의한다.
통신이 끊기거나 잘못된 메시지가 들어와도 2호기가 정지 상태를 유지하는 것을 우선한다.

## 2. 공통 환경

| 항목 | 규약 |
|---|---|
| ROS 버전 | ROS 2 Jazzy |
| 도메인 | `ROS_DOMAIN_ID=10` |
| 호기 번호 | `unit=2` |
| 호기 간 메시지 | `std_msgs/String` 안의 JSON |
| 문자 인코딩 | UTF-8 |
| 시간 `t` | Unix 시간, 초 단위 실수 |
| 기본 QoS | Reliable, Volatile, Depth 10 |
| 상태 송신 | 5 Hz |
| 명령 제한 시간 | 1.0초 |
| 추종 상태 제한 시간 | 2.5초 |

모든 호기 간 JSON 메시지는 숫자형 `t`를 포함한다. `t`는 기록과 비교용이며 안전 정지 시간은 2호기의 실제 수신 시각으로 계산한다.

## 3. 호기 간 통신 흐름

```text
1호기
  fleet/state
  fleet/cargo/target
  fleet/cmd/unit2
        ↓
2호기 fleet/follower
        ↓
mission 판단 → drive 중재 → base 구동
        ↓
2호기 fleet/status/unit2
        ↓
1호기
```

## 4. 호기 간 토픽

| 토픽 | 방향 | 타입 | 주기 | 역할 |
|---|---|---|---|---|
| `/fleet/state` | 1호기 → 전체 | String JSON | 권장 5 Hz | 계절, 모드, 전체 진행 상태 |
| `/fleet/cargo/target` | 1호기 → 2호기 | String JSON | 감지 중 권장 5 Hz | 여름 화물 위치 |
| `/fleet/cmd/unit2` | 1호기 → 2호기 | String JSON | 동작 중 권장 5 Hz | 2호기 행동 명령 |
| `/fleet/cmd_vel/unit2` | 1호기 → 2호기 | Twist | 결합주행 중 50 Hz | 강체 결합체 속도 |
| `/fleet/status/unit2` | 2호기 → 1호기 | String JSON | 5 Hz | 동작, 센서, 구동 상태 |
| `/fleet/cmd_vel/unit2` | 1호기 → 2호기 | Twist | **50 Hz** | 결합 이동 속도. 아래 "결합 이동" 절 |

## 5. 전체 상태 메시지

토픽: `/fleet/state`

```json
{
  "t": 1785891600.125,
  "section": "summer",
  "phase": "running",
  "mode": "auto",
  "leader_stopped": false
}
```

| 필드 | 형식 | 필수 | 설명 |
|---|---|---|---|
| `t` | number | 필수 | 송신 시각 |
| `section` | string | 필수 | 현재 계절 또는 호송 구간 |
| `phase` | string | 선택 | 전체 진행 상태, `finished`이면 공용 추종 종료 |
| `mode` | string | 필수 | `auto` 또는 `manual` |
| `leader_stopped` | boolean | 선택 | 1호기 정지 상태, 기본값 `false` |
| `leader_linear` | number | 선택 | 1호기 현재 전진 속도(m/s). 결합 이동 전용 |
| `leader_angular` | number | 선택 | 1호기 현재 회전 속도(rad/s). 결합 이동 전용 |

> **v1.1에서 `leader_linear` / `leader_angular` 를 뺐다.** 결합 이동 속도는
> `/fleet/state` 가 아니라 전용 토픽 `/fleet/cmd_vel/unit2` 로 나른다.
> `/fleet/state` 는 5 Hz 상태 알림이라 50 Hz 속도 명령을 나를 수 없고,
> 1호기 `fleet/protocol.py:make_state` 는 애초에 이 필드를 실은 적이 없다.

허용 `section` 값은 `spring`, `summer`, `autumn`, `winter`, `escort`이다.

여름 구간은 개별 행동 명령을 사용한다. 그 외 구간은 `phase`가 `finished`가 아니면 2호기가 자동으로 공용 추종을 선택한다.

## 6. 2호기 행동 명령

토픽: `/fleet/cmd/unit2`

```json
{
  "t": 1785891601.250,
  "seq": 42,
  "action": "approach"
}
```

적재 명령은 롤러 값을 포함한다.

```json
{
  "t": 1785891604.500,
  "seq": 43,
  "action": "load",
  "roller": 0.7
}
```

| 필드 | 형식 | 필수 | 설명 |
|---|---|---|---|
| `t` | number | 필수 | 송신 시각 |
| `seq` | integer | 필수 | 0 이상의 명령 번호 |
| `action` | string | 필수 | 수행할 행동 |
| `roller` | number | `load`만 필수 | 롤러 명령, -1.0 이상 1.0 이하 |

### 행동 값

| action | 의미 | 2호기 동작 |
|---|---|---|
| `idle` | 대기 | 주행과 롤러 정지 |
| `follow` | 선도 추종 | ArUco 마커와 UWB 추종 |
| `approach` | 화물 접근 | 목표 거리까지 접근 |
| `load` | 화물 적재 | 저속 후진과 롤러 구동 |
| `hold` | 화물 고정 | 롤러 정지, 적재 완료 처리 |
| `release` | 화물 배출 | 역방향 롤러 구동 |
| `stop` | 안전 정지 | 모든 주행 출력 차단 |

### 순번과 반복 송신

- 새 동작을 보낼 때 `seq`를 1 증가시킨다.
- 같은 `seq`의 반복 메시지는 행동을 다시 실행하지 않지만 통신 생존 신호로 인정한다.
- 일반 명령에서 이전 `seq`보다 작거나 같은 값은 무시한다.
- `stop`은 `seq`가 오래됐거나 중복되어도 즉시 적용한다.
- 여름 임무와 개별 행동 중에는 1초 안에 명령이 다시 수신되어야 한다.

## 7. 화물 목표 메시지

토픽: `/fleet/cargo/target`

```json
{
  "t": 1785891602.000,
  "detected": true,
  "distance_mm": 650.0,
  "lateral_mm": -25.0,
  "confidence": 0.92
}
```

| 필드 | 형식 | 필수 | 설명 |
|---|---|---|---|
| `t` | number | 필수 | 송신 시각 |
| `detected` | boolean | 필수 | 화물 감지 여부 |
| `distance_mm` | number 또는 null | 선택 | 목표 접근 거리 |
| `lateral_mm` | number 또는 null | 선택 | 화면 중심 기준 좌우 오차 |
| `confidence` | number 또는 null | 선택 | 검출 신뢰도 |

`detected=false`일 때 나머지 측정값은 생략하거나 `null`로 보낸다.

## 8. 2호기 상태 메시지

토픽: `/fleet/status/unit2`

```json
{
  "t": 1785891604.700,
  "unit": 2,
  "phase": "loading",
  "seq_ack": 43,
  "uwb_mm": null,
  "current_a": null,
  "roller": 0.7,
  "red_detected": true,
  "marker_detected": false,
  "heading_deg": null,
  "solenoid_locked": false,
  "action": "load",
  "watchdog_stopped": false
}
```

| 필드 | 형식 | 설명 |
|---|---|---|
| `t` | number | 2호기 상태 생성 시각 |
| `unit` | integer | 항상 2 |
| `phase` | string | 현재 임무 단계 |
| `seq_ack` | integer | 마지막으로 수락한 명령 번호 |
| `uwb_mm` | number 또는 null | UWB 거리 |
| `current_a` | number 또는 null | 좌우 모터 중 최대 절대 전류 |
| `roller` | number | 현재 롤러 명령 |
| `red_detected` | boolean | 적색 화물 마커 감지 여부 |
| `marker_detected` | boolean | ArUco 추종 마커 감지 여부 |
| `heading_deg` | number 또는 null | BNO086 방위각 |
| `solenoid_locked` | boolean | 솔레노이드 동작 상태 |
| `action` | string | 현재 행동 |
| `watchdog_stopped` | boolean | 통신 시간 초과 정지 여부 |

UWB와 BNO086이 미장착된 현재 상태에서는 `uwb_mm`과 `heading_deg`가 `null`인 것이 정상이다. CAN 미연결 시 `current_a`도 `null`일 수 있다.

## 9. 임무 단계

허용되는 2호기 상태 단계는 다음과 같다.

```text
idle
following
approach
search
turning
backing
contact
loading
loaded
locked
error
```

여름 화물 적재의 기본 진행 순서는 다음과 같다(`loaded`가 적재 완료 상태이며,
`locked`는 자동 결합(coupling) 임무 전용 단계다).

```text
approach → search → turning → backing → contact → loading → loaded
```

## 10. 2호기 내부 ROS 토픽

> **네임스페이스 (2026-08-13 변경)**
>
> 아래 내부 토픽은 전부 호기 네임스페이스 아래에 있다. 표의 `/cmd_vel` 은
> 실제로는 `/unit2/cmd_vel` 이며, 코드에서는 앞
> 슬래시가 없는 **상대 이름** `cmd_vel` 로 쓴다.
>
> 1호기와 `cmd_vel` / `roller_cmd` / `motor/status` / `solenoid_cmd` 가 같은
> 이름으로 겹쳐, 같은 도메인에 올라오면 1호기 주행 명령이 2호기 모터로 들어가기
> 때문이다.
>
> **4장의 호기 간 토픽은 예외다.** `/fleet/state`, `/fleet/cargo/target`,
> `/fleet/cmd/unitN`, `/fleet/status/unitN`, `/fleet/cmd_vel/unitN` 은 1호기와
> 공유하므로 절대 경로를 유지한다. 이 규약의 계약 부분은 바뀌지 않았다.
>
> 또한 `/fleet/action` 은 arbiter 가 모드에 따라 명령 출처를 고른 뒤 내보내는
> `mission/action` 으로 대체됐다. 미션 노드는 `mission/action` 만 구독한다.
> 운용 UI 가 자동 모드에서 보내는 요청은 `mission/request` 다.

### 영상과 센서

| 토픽 | 타입 | 발행 | 구독 | 역할 |
|---|---|---|---|---|
| `/camera/image/compressed` | CompressedImage | camera | marker, cargo, UI | 원본 카메라 영상 |
| `/vision/image/compressed` | CompressedImage | marker_vision | 진단 화면 | 마커 표시 영상 |
| `/marker/pose` | PoseStamped | marker_vision | follow_leader | 마커 위치와 자세 |
| `/marker/status` | String JSON | marker_vision | fleet, UI | 마커 검출 상태 |
| `/marker/enable` | Bool | 운용 노드 | marker_vision | 마커 검출 사용 여부 |
| `/marker/command` | String JSON | 운용 노드 | marker_vision | 보정, 활성화 명령 |
| `/uwb/distance_mm` | Float32 | uwb | follow, cargo, fleet | 거리 단위 mm |
| `/uwb/status` | String JSON | uwb | UI | UWB 준비 상태 |
| `/imu/heading_deg` | Float32 | imu | cargo, fleet | 방위각 단위 degree |
| `/imu/status` | String JSON | imu | UI | IMU 준비 상태 |

`/marker/pose`의 `position.x`는 화면 중심 기준 좌우 오차 m, `position.z`는 거리 m이다. 양의 `position.x`는 마커가 화면 오른쪽에 있다는 뜻이다.

### 임무와 주행

| 토픽 | 타입 | 발행 | 구독 | 역할 |
|---|---|---|---|---|
| `/fleet/action` | String JSON | follower | mission, arbiter | 검증된 내부 행동 명령 |
| `/cmd_vel/follow` | Twist | follow_leader | arbiter | 추종 속도 후보 |
| `/cmd_vel/cargo` | Twist | cargo_load | arbiter | 적재 속도 후보 |
| `/cmd_vel/coupling` | Twist | coupling | arbiter | 자동 결합 속도 후보 |
| `/cmd_vel/manual` | Twist | manual | arbiter | 수동 속도 후보 |
| `/cmd_vel` | Twist | arbiter | motor | 최종 모터 속도 명령 |
| `/drive/mode_cmd` | String | 운용 노드 | arbiter | `auto`, `manual`, `reset_emergency` |
| `/drive/status` | String JSON | arbiter | UI | 최종 주행 상태 |
| `/mission/phase` | String | arbiter | fleet | 현재 임무 단계 |
| `/mission/follow/status` | String JSON | follow_leader | arbiter | 추종 준비와 출력 상태 |
| `/mission/cargo/status` | String JSON | cargo_load | arbiter, UI | 적재 단계와 출력 상태 |
| `/coupling/cmd` | String JSON | 시험 UI | coupling | 결합 시작, 취소, 기준 재저장 |
| `/mission/coupling/status` | String JSON | coupling | arbiter, UI | 결합 가능 여부와 진행 단계 |

최종 `/cmd_vel`은 `drive/arbiter.py`만 발행한다. 다른 주행 노드는 후보 토픽만 발행해야 한다.

### 구동과 안전

| 토픽 | 타입 | 발행 | 구독 | 역할 |
|---|---|---|---|---|
| `/motor/status` | String JSON | motor | fleet, cargo, coupling, UI | CAN 응답시간, 전류 순번, 공용 접촉 기준 |
| `/roller_cmd` | Float32 | cargo_load | motor | 정규화 롤러 명령 |
| `/cmd_vel/coupled` | Twist | coupled_drive | arbiter | 결합 이동 속도 후보 |
| `/mission/coupled/status` | String JSON | coupled_drive | UI | 결합 이동 상태와 정지 사유 |
| `/solenoid_cmd` | Bool | coupling | solenoid | 결합 잠금과 해제 명령 |
| `/solenoid_status` | Bool | solenoid | coupling, fleet, UI | 솔레노이드 동작 상태 |
| `/cargo/red_detected` | Bool | cargo_load | fleet | 적색 화물 감지 상태 |
| `/safety/stop` | Bool | follower | cargo, arbiter | 통신 안전 정지 |
| `/safety/emergency_stop` | Bool | 시험 UI | arbiter | 운용자 비상 정지 |

자동 결합은 모터 전류가 설정값을 연속 3회 넘으면 정지한 뒤 솔레노이드를 잠근다.

### 자동 결합 명령

토픽: `/coupling/cmd`

```json
{"action":"start"}
```

| action | 역할 |
|---|---|
| `start` | 저장된 시동 기준으로 자동 결합 시작 |
| `cancel` | 진행 중인 결합만 중단 (솔레노이드는 잠긴 채로 둔다) |
| `unlock` | 솔레노이드 해제 — **사람이 명시적으로 누를 때만** 하는 유일한 경로다 (2026-09-03 분리) |
| `recapture` | 현재 기준을 버리고 ArUco 마커를 다시 1초간 측정 |

시동 후 네 마커가 1초 연속 검출되면 `span_px`, `lateral`, `roll`, `pitch`, `yaw`의 중앙값을 메모리에 저장한다. 인식이 순간적으로 끊겨도 1초 동안 버튼 상태를 유지하며, 1초 이상 끊기면 결합 시작을 막는다. 기준은 노드가 재시작되면 다시 저장한다. 결합 주행 중에는 마커가 0.35초 끊기면 이전 자세로 계속 움직이지 않고 정지한다.

정렬 단계의 선형 속도는 기준 `span_px`와 현재 크기의 비율을 거리 오차로 환산한 뒤 이전 결합기의 PID 값으로 제어한다. `target_distance_m`, `linear_kp`, `linear_ki`, `linear_kd`와 적분 제한, 미분 필터 계수는 `mission/config/coupling.yaml`에서 조정한다. 회전 속도는 lateral과 yaw 오차의 P 제어를 사용하며, `/mission/coupling/status`의 `linear_pid`에서 거리 오차와 P·I·D 항, 제한 후 출력을 확인한다.

```text
reference_wait → ready → align → contact → locking → locked
```

결합 시작 조건은 기준 저장, 안정된 ArUco 마커, CAN 연결, 최신 모터 전류, 양수인 접촉 전류 기준, 안전 정지 해제다. 접촉 전류 기준은 적재와 결합이 **따로** 가진다(2026-08-31 분리) — 결합은 `mission/config/coupling.yaml`의 `contact_current_threshold_a`(1.7A, 0.10 m/s 실측), 적재는 `mission/config/cargo.yaml`의 `unit2_load_high_current_a`/`unit2_load_low_current_a`다. 속도를 바꾸면 그 속도에서 기준을 다시 재야 한다. 오류가 발생하면 주행 중재기가 0 속도를 유지하며 운용자가 취소해야 일반 주행으로 돌아간다.

`/motor/status`의 `can_connected`는 CAN 장치 파일 존재 여부가 아니라 0.5초 이내 실제 모터 응답 여부다. `can_response_age_s`는 마지막 정상 응답 경과시간이며 `current_seq`는 새 전류 응답마다 증가한다. 적재와 결합은 서로 다른 전류 응답 3회만 접촉 판정에 사용한다.

## 11. 안전 우선순위

```text
비상 정지
통신 안전 정지
명령 또는 센서 시간 초과
선택된 주행 명령
```

- `/safety/emergency_stop=true`이면 최종 속도는 즉시 0이다.
- `/safety/stop=true`이면 최종 속도와 롤러 출력은 0이다.
- 선택된 속도 후보가 0.5초 이상 갱신되지 않으면 최종 속도는 0이다.
- 추종 중 마커 또는 UWB가 0.5초 이상 끊기면 추종 출력은 0이다.
- 추종 중 `/fleet/state`가 2.5초 이상 끊기면 통신 안전 정지한다.
- 여름 임무 명령이 1.0초 이상 끊기면 통신 안전 정지한다.

## 12. 시험 화면 통신

시험 실행 명령은 다음과 같다.

```bash
ros2 launch mars_launch test_drive.launch.py
```

접속 주소는 `http://10.10.2.42:5000`이다.

| HTTP 경로 | 방식 | 역할 |
|---|---|---|
| `/` | GET | 시험 화면 |
| `/api/status` | GET | 통합 상태 JSON |
| `/camera.mjpg` | GET | 카메라 MJPEG |
| `/api/emergency-stop` | POST | 비상 정지 설정 |
| `/api/reset-emergency` | POST | 비상 정지 해제 |
| `/api/coupling/start` | POST | 자동 결합 시작 |
| `/api/coupling/cancel` | POST | 결합 취소 또는 잠금 해제 |

시험 화면은 일반 수동 주행 명령을 제공하지 않으며 자동 결합과 안전 정지만 제공한다.

## 13. 메시지 수락 조건

다음 메시지는 폐기하고 현재 안전 상태를 유지한다.

- JSON 객체가 아닌 메시지
- 숫자형 `t`가 없는 메시지
- 허용 목록에 없는 `section`, `mode`, `action`
- 음수 또는 정수가 아닌 `seq`
- `roller`가 없거나 -1.0에서 1.0 범위를 벗어난 `load` 명령
- 숫자로 변환할 수 없는 화물 거리와 신뢰도

## 14. 규약 변경 원칙

- 호기 간 토픽명과 JSON 필드는 1호기와 2호기에서 동시에 변경한다.
- 필드를 추가할 때는 기존 수신기가 무시할 수 있도록 선택 필드로 먼저 배포한다.
- 필드를 삭제하거나 의미와 단위를 바꿀 때는 규약 버전을 올린다.
- 거리 단위는 이름에 `_mm` 또는 `_m`, 각도 단위는 `_deg` 또는 `_rad`를 붙인다.
- 안전 정지와 `stop` 명령은 다른 기능보다 항상 우선한다.

## 결합 이동 (2026-08-31 추가, 2026-09-04 채널 교체)

결합이 `locked` 된 뒤 1호기와 함께 이동하는 구간의 규약이다. 여름은 적재를
제외한 구간 전체, 겨울은 제설 구간이 여기 해당한다.

2호기는 결합 중 **거리 제어를 하지 않는다.** 결합 로드가 거리를 물리적으로
고정하므로 추종(`follow_leader`)의 거리 제어기를 그대로 쓰면 로드와 싸운다.
대신 1호기의 속도를 그대로 복제한다.

### 채널: `/fleet/cmd_vel/unit2` (Twist, 50 Hz)

1호기 `drive/coupled.py` 가 **자기 구동 명령과 같은 루프에서** 이 토픽을 낸다.
같은 값을 같은 순간에 두 호기가 받아야 네 개 구동축이 어긋나지 않기 때문이다.

QoS 는 양쪽이 **정확히 같아야 한다** — `BEST_EFFORT` / `KEEP_LAST` depth 1 /
`VOLATILE`. 하나라도 다르면 ROS 2 는 연결을 맺지 않고 **아무 경고도 내지
않는다.** 밀린 속도 명령을 뒤늦게 따라가는 것이 위험하므로 최신 한 건만 본다.

> **왜 `/fleet/state` 가 아닌가** (2026-09-04): v1.0 은 `/fleet/state` JSON 에
> `leader_linear` 를 실어 보내기로 했지만, 1호기 `fleet/protocol.py:make_state`
> 는 그 필드를 실은 적이 없다. 두 호기가 서로 다른 채널로 말하고 있었고 2호기는
> 계속 `leader_velocity_missing` 으로 0을 냈다. 게다가 `/fleet/state` 는 5 Hz
> 상태 알림이라 실려 있었더라도 결합 주행 명령으로는 10배 느리다.

### 규칙

- 2호기 모드는 `coupled` 다. `drive/mode_cmd` 로 전환한다.
- `/fleet/cmd_vel/unit2` 가 `leader_timeout_s`(기본 0.5초) 안에 갱신되지 않으면
  2호기는 **0을 낸다.** 마지막 속도로 계속 가지 않는다. 50 Hz 기준 0.5초는
  25 프레임을 놓친 뒤라 통신 지터가 아니라 확실한 두절이다.
- 결합이 `locked` 가 아니면 2호기는 굴리지 않는다(`require_locked`).
- 1호기도 대칭으로 잠근다 — 2호기 status 가 끊기거나 `locked` 가 아니거나
  솔레노이드가 잠기지 않았으면 0을 보낸다.

### 조향: 강체이므로 복제한다 (2026-09-04 확정)

결합부가 **강체**로 확인되어 2호기도 1호기와 같은 각속도를 낸다
(`follow_angular: true`).

강체로 붙으면 두 호기는 하나의 긴 물체이고, 강체의 각속도는 어느 지점에서나
같다. 좌우를 같은 속도로 굴리면(조향 0) 2호기는 회전을 막는 쪽으로 버텨서
1호기가 그 저항까지 이겨야 한다 — 잠긴 캐스터와 같다.

**힌지형으로 바뀌면 반드시 다시 꺼야 한다.** 조인트 각을 모르는 채로 조향을
복제하면 잭나이프가 난다.

**부호 주의**: 적재 과정에서 2호기는 180도 회전해 후면을 1호기 쪽으로 두므로,
1호기의 전진이 2호기에서는 후진일 수 있다. 회전도 마찬가지다. 2호기
`coupled.yaml` 의 `linear_sign` / `angular_sign` 이 그 보정값이며, 실측 전에는
낮은 속도로 부호부터 확인한다.
