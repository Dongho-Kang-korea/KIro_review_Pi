# cargo_load — `src/mission/mission/summer/cargo_load.py`

**최신화**: 2026-09-03

## 역할

여름 화물 적재 전체 단계(접근 → 탐색/회전 → 후진 접촉 → 적재 → 고정)를
자기완결형으로 진행한다. 계절과 무관하게 **항상 떠 있고**, 명령이 없으면
가만히 있는다(여름에만 실제로 명령이 온다). **v1.2부터**(2026-08-31,
[TODO.md](../../TODO.md) 18번) 1호기가 `turn`/`couple` 명령으로 일부
단계를 직접 지휘할 수도 있다 — 자기완결형 경로와 공존한다(아래 "동작
요약" 참고).

## 실행

`mars_launch/robot.launch.py`에서 항상 뜬다.

## 설정

`src/mission/config/cargo.yaml` — 단계별 속도(`approach_speed_mps`,
`turn_speed_rps`, `back_speed_mps`, `load_speed_mps`, `release_roller`),
허용오차(`approach_tolerance_mm`, `turn_target_deg`/`turn_tolerance_deg`),
`current_confirm_samples`, 적색 HSV 범위(`red_h_min_1/2`, `red_h_max_1/2`,
`red_s_min`, `red_v_min`, `red_min_area`), `red_detect_rate_hz`(기본 5.0,
2026-09-01 추가 — 아래 "동작 요약" CPU 부하 참고). 숫자는 이 파일이
최신 출처다.

## 토픽

- 구독: `mission/action` (arbiter 재발행), `/fleet/cargo/target` (절대경로,
  1호기가 표적 알려줌), `uwb/distance_mm`, `imu/heading_deg`, `motor/status`,
  `safety/stop`, `camera/image/compressed`, `roller_manual_cmd`
  (Float32 — 웹 콘솔 수동 컨베이어 조작, 2026-08-28 추가)
- 발행: `cmd_vel/cargo` (Twist, arbiter의 SOURCES 중 하나), `roller_cmd`
  (Float32), `cargo/red_detected` (Bool),
  `mission/cargo/status` (String JSON — `roller_manual_active` 필드로 지금
  수동/자동 어느 쪽이 롤러를 몰고 있는지 알 수 있다)

## 동작 요약 (phase 상태기계)

**자기완결형 경로**(기존, 1호기 명령 없이도 진행): `idle` →
(`action=approach`) → `approach`(UWB 거리로 `/fleet/cargo/target` 접근) →
`search`(적색 검출 대기) → `turning`(IMU heading 180도 회전) →
`backing`(후진, 전류 접촉 확인) → `contact` → (`action=load`) → `loading`
(롤러 정방향) → (`action=hold`) → `loaded`(롤러 0, 적재 완료 — 종단 상태).
`action=release`는 롤러 역방향으로 화물을 배출한다. 별도 잠금장치(서보 등)는
쓰지 않기로 결정했다(2026-08-30, [TODO.md](../../TODO.md) 참고) — `loaded`에
도달하면 그대로 완료로 본다. `loaded` 상태에서 `action=approach`를 다시
받으면 새 사이클을 시작할 수 있다.

**1호기 지휘 경로**(v1.2, 2026-08-31 추가 — [TODO.md](../../TODO.md)
18번): `turn`(`angle_deg` 동반)과 `couple` 두 액션을 새 상태·경로 없이
기존 로직에 얹었다.

- `turn`: `idle`/`error`/`loaded`에서만 받는다. **`search`에서 적색
  검출로 들어가는 것과 같은 `turning` phase 로직**(IMU heading으로
  `angle_delta` 비교하며 `turn_speed_rps`로 회전)을 재사용하되, 목표각을
  yaml 고정값(`turn_target_deg`) 대신 명령의 `angle_deg`로 받는다
  (`self.turn_target_deg` 인스턴스 속성, 부호는 무시하고 크기만 씀).
  완료되면 자기완결형 경로처럼 `backing`으로 이어지지 않고 **`idle`로
  돌아가 1호기의 다음 명령을 기다린다** — 어느 경로로 들어왔는지는
  `self.turn_via_command`로 구분한다.
- `couple`: **이 노드는 아무것도 하지 않는다.** 2026-09-03부터
  [coupling](coupling.md)이 `fleet/action`을 직접 구독해 `couple`을
  스스로 받는다(TODO 24번) — 결합은 겨울에도 쓰는데 여름 노드를
  거치면 계절별 게이팅이 생겼을 때 끊기기 때문이다. 예전에는 여기서
  `coupling/cmd`에 `{"action":"start"}`를 발행했다.
  정렬·접촉·잠금은 원래부터
  정렬·접촉·잠금은 [coupling](coupling.md)의 몫이고, 거기서 자체
  eligibility를 판단한다. **1호기가 미리 `turn`으로 방향을 잡아 둔 뒤
  `couple`을 보내는 걸 전제한다** — `couple` 자체는 회전하지 않는다(팀원
  워크스페이스 `~/ros2_ws`는 `couple` 안에 회전까지 묶여 있었지만, 이
  저장소는 `turn`이 이미 독립 명령이라 다시 합치지 않기로 했다 — 1호기
  쪽과 실제 시퀀스 합의가 필요한 지점이니 통합 시험 전에 확인할 것).
  결합 진행 상태는 이 노드가 따로 보고하지 않는다 — `drive/arbiter`의
  `_phase()`가 `coupling_status.phase`를 이미 `approach`/`contact`/
  `locked`/`error` 어휘로 매핑해 `/fleet/status/unitN.phase`로 그대로
  내보낸다.

- 적색 검출은 HSV 두 구간(0~12°, 165~179°, 빨강이 hue 경계를 걸치므로)
  마스크 합집합 + 최소 면적(`red_min_area`) 검사로 판정한다 — YOLO를 쓰지
  않는다.
- 접촉 판정 기준값은 **`coupling.py`와 따로 관리한다**(2026-09-01).
  `unit2_load_high_current_a`/`unit2_load_low_current_a`는 2호기가
  `load_speed_mps`로 후진하며 적재물을 밀 때의 **주행 모터(ID 1·2)**
  전류다 — 롤러(ID 3)는 커밋 `37151d5`에서 분리돼 이 판정에 안
  들어간다. 저항이 `high` 이상 올랐다가 `low` 아래로 떨어지면 완료로
  본다(제약 `0 < low < high`). 2026-09-03에 1.3/1.0을 넣었으나
  **실측 전 임시값**이다(TODO 2번). 다만
  실제로는 `cargo.yaml`에 `contact_current_threshold_a` 키 자체가 없다
  (2호기 쪽 화물 접촉 판정은 이 노드가 아니라 **1호기**가 자기 config로
  하고, 그 결과를 `/fleet/status/unit1`의 `cargo_contact_detected`로
  보내며, 2호기는 `contact_uwb_threshold_mm` 거리 안일 때만 그 판정을
  인정한다). 결합(`coupling.yaml`)의 `contact_current_threshold_a`(1.7A,
  확정)와는 완전히 별개 값이며 서로 공유하지 않는다 — 적재는 결합보다
  느리게 접근해 접촉 전류가 낮으므로(1.2~1.4A대 관측) 값을 공유하면
  한쪽이 반드시 틀린다. 고정 속도(`back_speed_mps`)로 후진하며 1호기
  판정이 연속 `contact_confirm_samples`회 들어오는지만 본다(PID 아님).
- `safety_stop`이면 롤러도 강제로 0.
- **적색 검출 CPU 부하 완화(2026-09-01)**: 예전엔 카메라 프레임(30fps)마다
  1280x720 JPEG 디코딩+HSV+컨투어를 전부 돌려 코어 하나의 절반을
  미션과 무관하게 항상 먹었다(실측: cargo_load 52%, 전체 load 17/4코어).
  화물은 빠르게 움직이지 않으므로 `red_detect_rate_hz`(기본 5Hz)로
  판정 주기를 솎는다 — `cargo/red_detected` 계약은 그대로다. 같은 이유로
  모듈 임포트 시 `cv2.setNumThreads(1)`도 호출한다([camera](../base/camera.md)
  참고).
- **수동 컨베이어 조작**(2026-08-28): `roller_manual_cmd`는 `action`이
  `idle`/`stop`일 때만(자동 미션이 안 도는 중) 채택되고,
  `manual_roller_timeout_s`(기본 0.4s) 안에 새 값이 없으면 자동으로
  0이 된다 — 자동 미션이 시작되는 순간 즉시 자동 쪽이 우선한다. 웹
  콘솔의 "컨베이어 수동 조작" 패널이 이 토픽을 20Hz로 재발행한다
  ([console 문서](../ui/console.md) 참고).

## 연결

- 위: `drive/arbiter`(`mission/action`), 1호기(`/fleet/cargo/target`),
  `base/uwb`, `base/imu`, `base/motor`(`motor/status`), `base/camera`.
- 아래: `drive/arbiter`(`cmd_vel/cargo`), `base/motor`(`roller_cmd` —
  독립 소유권), `mission/coupling`(`coupling/cmd`의 `start` — 2026-08-31
  추가, `couple` 액션 수신 시에만).
- 접촉 전류 기준은 `mission/coupling`과 **공유하지 않는다**(위 "동작
  요약" 참고) — [HARDWARE.md](../../HARDWARE.md) "접촉 전류 기준값" 참고.

## 구현 상태

완성(코드 기준). 실물 검증은 [HARDWARE.md](../../HARDWARE.md)의 여름 적재
3단계 하드웨어 요구사항(UWB/BNO086/전류실측)이 갖춰져야 끝까지 진행된다.
화물 고정 단계는 서보 없이 `hold` 액션만으로 완료된다.

## 알려진 이슈

미장착 하드웨어 관련 항목은 [TODO.md](../../TODO.md)에서 확인한다.
수동 컨베이어 병합 로직은 `src/mission/test/test_cargo_load_manual_roller.py`로
자동화 테스트했다(2026-08-28, 5개 통과). `turn`/`couple` 배선은
`src/mission/test/test_cargo_load_turn_couple.py`로 자동화 테스트했다
(2026-08-31, 7개 통과 — 자기완결형 `search`→`turning`→`backing` 경로가
그대로 유지되는지 회귀 확인 포함). **아직 안 된 것**: 1호기가 실제로
`turn`/`couple`을 이 순서·타이밍으로 보내는지, `couple`을 `turn` 없이
보내면 어떻게 되길 기대하는지는 1호기 쪽(`comp-upgrade-20260830`
브랜치, 아직 origin 미push)이 정리되고 실물 통합 시험을 해야 확인된다
([TODO.md](../../TODO.md) 18번).
