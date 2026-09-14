# 하드웨어 배선/결선값

**최신화**: 2026-09-03

`docs/`의 노드 문서보다 한 단계 위 — "실제로 뭐가 어디에 꽂혀 있고 실측값이
뭔지"를 모은다. 코드/로직 설명은 [docs/](docs/README.md)를 본다. 이 문서는
[AGENTS.md](AGENTS.md) 2절 정책에 따라 `src/base/config/*.yaml` 또는
`src/base/base/*.py`가 바뀌면 같이 갱신한다.

## 한눈에 보기

| 장치 | 상태 | 결선 | 문서 |
|---|---|---|---|
| CAN 모터(좌/우/롤러) | 장착·배선 완료 | CAN0, PCAN-USB, ID 1/2/3 | [motor](docs/base/motor.md) |
| Picamera2 | 장착·배선 완료 | libcamera, rotation 180 | [camera](docs/base/camera.md) |
| ArUco 마커 인식 | **캘리브레이션 확정, pose 실측 검증 통과** | 카메라 영상 기반(별도 배선 없음) | [marker_vision](docs/base/marker_vision.md) |
| 솔레노이드(결합용 잠금) | 장착·배선 완료. **2026-09-01 GPIO 극성 재정정(4번째) + 자동 재잠금 타이머 추가, 실물 재검증 필요** | GPIO23, gpiochip4, **active_low: false**(2026-09-01 정정 — 잠금=GPIO LOW/무통전, 해제=GPIO HIGH/통전) | [solenoid](docs/base/solenoid.md) |
| BNO086 IMU | **장착·I2C 확인됨**, 영점 미교정. 아루코 기준 360도 회전 검증 5회 완료(오프셋 값은 아직 0.0) | I2C, 주소 0x4B(75) | [imu](docs/base/imu.md) |
| DWM1001 UWB | **장착·활성화됨(2026-08-29), 오프셋 정밀 보정 남음** | 시리얼(2026-09-01부터 `/dev/serial/by-id/usb-SEGGER_J-Link_000760046619-if00` 고정 경로 — 모듈 3개 중 1개를 태그로 전환) | [uwb](docs/base/uwb.md) |

## CAN 모터

- 인터페이스: `can0` (SocketCAN), 어댑터: PCAN-USB — TX 큐가 짧아
  `can_tx_interval_s`(기본 0.02s)로 프레임 전송을 직렬화한다
  (`base/config/motor.yaml`).
- 모터 컨트롤러: ODrive/Steadywin, CANSimple 프로토콜.
- **주기 송신은 `0x01`(heartbeat 1Hz)과 `0x09`(엔코더 100Hz) 둘뿐이다.**
  전류(`0x14`)는 RTR로 요청해야 응답한다 — `motor.yaml` 의
  `enable_rtr_requests: true` 가 그 요청을 담당한다 (2026-08-31 실측 확인).
- 노드 ID: 좌=1, 우=2, 독립 롤러=3(`left_id`/`right_id`/`roller_id`).
- `require_can: false` — CAN 버스가 없어도 "안전 미연결 모드"로 기동한다
  (실물 없이 통신 시험 가능).
- **2026-09-01 어댑터 탈부착 자동 복구**: PCAN-USB가 접촉 불량으로
  빠졌다 붙으면(`dmesg`: `Rx urb aborted`, `can0 removed`) 소켓이 죽어
  `[Errno 19] No such device`가 난다. 예전에는 이 예외가 노드까지
  올라가 프로세스가 죽어서, 어댑터가 다시 붙어도 모터 제어가 영영
  안 돌아왔다(실측 확인). 지금은 죽은 버스를 놓고
  `can_reconnect_interval_s`(기본 2초)마다 스스로 재오픈·모터
  재초기화한다(`src/base/base/motor.py`).

## 카메라

- Picamera2(libcamera) 기반, `PYTHONPATH=/usr/local/lib/python3/dist-packages`
  주입이 필요하다(launch가 자동 처리).
- `rotation: 180`(장착 방향), `full_fov: true`(센서 전체 화각 사용).
- 해상도는 1280x720(16:9, crop 없음) — 2026-08-28에 ArUco pose 정확도
  때문에 640x360에서 올렸다.
- **`frame_rate: 30 → 15`(2026-09-01)**: 영상 노드(카메라·marker_vision·
  cargo_load)가 여러 개 동시에 돌아 Pi 5의 4코어에 부하가 몰려(실측
  load 15) 마커 검출률이 0%까지 떨어지는 걸 확인했다. 결합 접근 속도
  (0.10 m/s)·제어 주기(20Hz)·마커 타임아웃(0.35초)을 보면 15fps로도
  충분하다고 판단해 낮췄다. 같은 이유로 `cv2.setNumThreads(1)`을
  `camera.py`/`aruco_tracker.py`/`cargo_load.py`에 추가했다 — 영상
  노드가 프로세스 단위로 이미 병렬이라 OpenCV 내부 스레드를 1로
  묶는 편이 총 처리량이 낫다.
- `jpeg_quality: 70 → 82`(2026-09-01) — marker_vision이 이 JPEG를 직접
  검출하므로 경계 압축 손실을 줄였다.
- 카메라가 없거나 라이브러리 import 실패 시 더미 프레임으로 자동 폴백한다.
- ArUco pose(거리/자세) 캘리브레이션은 [calibrate_camera](docs/base/calibrate_camera.md)
  도구(체커보드를 노트북 화면에 띄우는 방식)로 완료·실측 검증까지
  끝났다 — `marker.yaml`에 반영돼 있다.

## BNO086 IMU

- I2C, 주소 `0x4B`(십진 75) — `/dev/i2c-1` 기준 실측 확인됨
  (`base/config/imu.yaml` 주석).
- `heading_offset_deg`는 아직 `0.0` — **영점 미교정**. 실측 후 이 값을
  갱신해야 `imu/heading_deg`가 실제 방위와 맞는다.
- 초기 SHTP batch 경고(RuntimeError)는 정상 동작 범위로 취급한다
  ([imu.md](docs/base/imu.md) 참고).
- **핀맵(물리 핀 번호, Pi 5 40핀 헤더 기준)** — `imu.py`가 Adafruit
  `board.SCL`/`board.SDA`(라즈베리파이 기본 I2C1)를 그대로 쓰는 것으로
  코드 확인됨. BCM→물리 핀 변환은 Pi 5 40핀 헤더 표준 배열 기준(신규
  실측 아님):
  - SDA → **BCM2 = 물리 핀 3**(I2C1 SDA)
  - SCL → **BCM3 = 물리 핀 5**(I2C1 SCL)
  - VIN(전원), GND → **미기록** — 코드/설정에 남지 않는 값이라 실측 필요
    (BNO086 브레이크아웃은 보통 3.3~5V 겸용이라 어느 레일에 물렸는지
    확인 전에는 3.3V 핀1/5V 핀2·4 중 단정하지 않는다)

## DWM1001 UWB — 장착·활성화(오프셋 정밀 보정 남음)

- `base/config/uwb.yaml`: `enabled: true`,
  `port: '/dev/serial/by-id/usb-SEGGER_J-Link_000760046619-if00'`
  (2026-09-01부터 — `/dev/ttyACM0`는 재부팅 시 열거 순서에 따라 번호가
  바뀔 수 있어 장치 고유 by-id 경로로 바꿨다), `baudrate: 115200`,
  `offset_mm: 0.0`(육안 확인만 됨, 정밀 보정 아직).
- **2026-09-01 시리얼 통신 견고화**: 셸 진입 명령(`\r\r`)을 한 틱에
  몰아 보내면 셸이 인터럽트/플로딩되는 문제가 있어
  `shell_enter_delay_s`(0.2초) 간격을 두고 나눠 보내도록 고쳤다.
  USB 쓰기가 간헐적으로 장치를 binary TLV 모드에 남겨 텍스트 응답이
  끊기는 문제에 대응해 `data_timeout_s`(3초)·`reconnect_interval_s`
  (2초)로 응답 두절을 감지하고 셸을 자동 재진입한다.
- 2026-08-29: 보유 모듈 3개(`DW4823`/`DWD00C`/`DW559E`)가 전부 공장
  기본값 앵커 상태였던 걸 확인 → `DW4823`을 셸 명령(`nmt`)으로 태그로
  전환, `DWD00C`는 앵커로 둔 채 `/dev/ttyACM0`에서 실측 거리 확인.
  `uwb.py`의 실제 버그 2개(능동 폴링 누락, m→mm 단위 누락)를 찾아 고침.
  이어서 정지 감지(`stat_det`)로 갱신 주기가 10초까지 느려지는 문제를
  발견해 `acts`/`aurs`/`reset`로 1초 고정 갱신으로 재설정(플래시 저장,
  전원 재인가에도 유지 확인) — 자세한 경과는 [uwb](docs/base/uwb.md),
  [TODO.md](TODO.md) 1번 참고.
- 남은 일: 줄자 등 정확한 기준 거리와 비교한 `offset_mm` 정밀 보정,
  여름 화물 적재 `approach` 단계 미션 통합 시험.
- 앵커(`DWD00C`)는 USB 전원만 쓰면 뽑는 순간 꺼진다 — 실사용 시 배터리 등
  별도 전원 필요. 추종(`follow_leader`)은 UWB 없어도
  `allow_marker_distance_fallback`로 동작하지만, 여름 화물 적재의
  `approach` 단계는 UWB가 필수다.

## 솔레노이드(자동 결합 잠금)

- GPIO23, `gpiochip4`(Pi 5 기본), **`active_low: false`**(2026-09-01
  정정 4 — 아래 참고. 그 전 정정 1~3의 경과는 아래에 그대로 남겨둔다).
- **정정 1(오전)**: `robot.launch.py`를 띄우자마자 솔레노이드가 잠기는
  증상 발견 → 실측(핀 3.3V=릴레이 통전, 0V=무통전)으로 `active_low`를
  `true`→`false`로 바꿈. 이때는 "통전 = 잠김"으로 가정했다.
- **정정 2(오후)**: 실제 커플링 걸쇠가 움직이는 방향을 직접 확인해보니
  반대였다 — 이 릴레이/솔레노이드는 **통전(HIGH)하면 걸쇠가 풀리고,
  무통전(LOW)이면 스프링으로 걸쇠가 잠기는 페일세이프 구조**다(전원이
  끊겨도 결합이 저절로 안 풀리게 하려는 설계로 보인다). 그래서
  `active_low`를 다시 `true`로 되돌렸다 — "통전 여부"와 "커플링 잠금
  여부"는 서로 다른 축이라는 걸 첫 정정 때 놓쳤다.
- **정정 3(2026-08-30, 기본값 수정)**: 그냥 밀어 넣으면 스프링 힘으로
  잠기는 게 "잠금"(무통전, 안전한 기본 상태)이고, 솔레노이드가 실제로
  걸쇠를 당겨 내리는 게 "해제"(통전)라는 걸 사용자가 다시 명확히
  설명해줬다. 그런데 코드는 시작·종료 시 "해제"로 초기화하고 있었다 —
  즉 기본 상태가 통전이었던 것이고, 이게 아래 과열 이슈의 직접 원인이었다.
  시작·종료 기본값을 "잠금"(무통전)으로 바꿨다(`src/base/base/solenoid.py`).
- **정정 4(2026-09-01)**: 실물에서 웹 콘솔 UI가 보여주는 잠금/해제
  상태와 실제 걸쇠 동작이 반대로 확인되어 `active_low`를 다시
  `true`→`false`로 뒤집었다. 논리 동작(잠금=무통전 스프링, 해제=통전
  당김)은 그대로이고, 그 논리를 GPIO23에 실어 내보내는 극성만 바꾼
  것이다 — 지금은 **GPIO LOW = 잠금(무통전)**, **GPIO HIGH = 해제
  (통전)**이다. 정정 3의 "과열 원인 수정"이 유효했는지는 이 정정으로
  다시 원점에서 재검증해야 한다(정정 3 당시 극성이 실제로는 반대였을
  수 있다는 뜻이라, 아래 과열 이슈의 재검증이 특히 중요하다).
- **자동 재잠금(2026-09-01 신규)**: `release_timeout_s`(기본 5초) —
  `solenoid_cmd`로 해제한 뒤 이 시간 안에 다시 잠금 명령이 안 오면
  자동으로 잠금(무통전)으로 되돌아간다. 해제 상태로 방치되어 코일이
  계속 통전되는(과열) 상황을 막는 안전장치다(`src/base/base/solenoid.py`).
- `enabled` 게이트가 없다 — 항상 활성(실측 배선 완료로 간주).
- **핀맵(물리 핀 번호, Pi 5 40핀 헤더 기준)** — `solenoid.yaml`의
  `gpio_pin: 23`은 BCM 번호(lgpio 기준). Pi 5 40핀 헤더 표준 배열로
  변환하면(신규 실측 아님, BCM→물리 핀 변환만):
  - 릴레이 신호선(IN) → **BCM23 = 물리 핀 16**
  - 릴레이 GND → 인접 GND 핀(예: 물리 핀 14/20 — 40핀 헤더 GND는 모두
    전기적으로 동일 노드)
  - 릴레이 모듈 VCC(코일측 전원) → **Pi 5V 레일**(물리 핀 2 또는 4)에서
    직접 공급(사용자 확인, 2026-08-29). 즉 솔레노이드 코일 자체도 5V
    소형 릴레이 모듈이 스위칭하는 구조 — 별도 12V/24V 외부 전원 없음.
- **2026-08-29 — 장시간 통전 과열 발견 → 2026-08-30 수정**: 평상시
  "해제"(기본값) 상태가 곧 코일 통전 유지 상태였던 게 원인 — 위 정정 3
  참고. 기본값을 "잠금"(무통전)으로 바꿔 근본 원인은 고쳤지만, 실물로
  과열이 실제로 사라졌는지·자동 결합 동작이 정상인지는 아직 재검증
  전이다 — 자세한 경위와 남은 확인 항목은
  [solenoid 문서](docs/base/solenoid.md) "알려진 이슈" 3번과
  [TODO.md](TODO.md)를 참고한다.

## 접촉 전류 기준값 — 결합은 확정, 적재는 미실측

- `base/config/motor.yaml`의 `contact_current_threshold_a: 0.0`은 공통
  폴백값으로 그대로 미실측(0)이다. 실제로는 미션별로 따로 잡으며
  그쪽 값이 우선한다:
  - `mission/config/coupling.yaml` → **`contact_current_threshold_a: 1.7`
    (2026-09-01 실측 확정)**. `contact_speed_mps: 0.10`에서 잰 값이라 두
    값은 짝이다 — 접근 속도를 바꾸면 전류 기준도 그 속도로 다시 재야
    한다(파일 내 주석 참고).
  - `mission/config/cargo.yaml` → 아직 미실측. 이 파일 자체에는
    `contact_current_threshold_a` 키가 없다(1호기가 자기 config로
    판정해 `cargo_load_current_dropped`로 보내는 구조라 2호기 쪽
    `cargo.yaml`에는 UWB 거리 게이트(`contact_uwb_threshold_mm`)만 있다).
  - 결합은 빠르게 붙어 전류가 크게 튀고(2A대 관측), 적재는 느리게 밀어
    낮다(1.2~1.4A대 관측) — 값 하나로는 두 단계를 못 가른다.
- 자동 결합(`align`→`contact`)은 위 1.7A 확정으로 자동 접촉 판정에
  진입한다. 여름 화물 적재 쪽 전류 판정(1호기 config)은 아직 `0.0`
  폴백에 머물러 있어 실측 전까지는 안전장치로 접촉 판정에 진입하지
  않는다 — 자세한 내용은 [TODO.md](TODO.md) 2번 참고.
