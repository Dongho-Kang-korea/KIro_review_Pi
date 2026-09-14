# coupling — `src/mission/mission/common/coupling.py`

**최신화**: 2026-09-03 (2026-09-01 방위각 기반 재작성 반영)

## 역할

ArUco 마커를 보며 좌우로 정렬한 뒤, 저속으로 접근해 1호기에 접촉(전류로
판정)하면 솔레노이드로 잠그고, 마지막으로 뒤로 당겨 하중이 실제로 걸렸는지
확인한다. 계절과 무관하게 항상 뜬다.

**2026-09-01 재작성**: 예전 화면 중심비(픽셀) 기반 PID 정렬을 걷어내고,
7월 `robot_control/docking_ctrl.py` 구조로 되돌아갔다 — 오차를 **방위각**
(`atan2(x_mm, z_mm)`, 마커까지의 좌우 각도)으로 재정의했다. 화면 중심비는
거리가 바뀌면 같은 어긋남도 값이 달라져 게인이 거리에 딸려 변하는데,
방위각은 그 왜곡이 없다. 자세한 실측 근거는 `coupling.py`/`coupling.yaml`
주석 참고.

## 실행

`mars_launch/robot.launch.py`, `test_drive.launch.py`에서 뜬다. `coupling/cmd`
발행자가 둘이다 — 시험 화면의 결합 버튼(수동), 그리고 이 노드 자신이
`fleet/action`의 `couple`을 직접 받아 시작한다(2026-09-03, TODO 24번 —
예전에는 여름 노드 [cargo_load](cargo_load.md)가 대신 발행했는데, 결합은
겨울에도 쓰므로 여름 노드를 거치면 안 된다)
(2026-08-31, [TODO.md](../../TODO.md) 18번) — 이 노드 자체는 호출 주체를
구분하지 않고 그대로 `start`로 처리한다. `start` 명령은 조건이 안 갖춰진
상태로 와도 버려지지 않는다 — `start_pending`으로 보관해 두고 `_tick()`이
매 tick(20Hz) eligibility를 재확인해 조건이 갖춰지는 순간 한 번만 시작한다.

## 설정

`src/mission/config/coupling.yaml` — 값과 실측 근거는 파일 주석이 최신
출처다. 실제로 제어에 쓰이는 주요 키:

- **정렬(align)**: `bearing_tolerance_deg`(기본 2도), `kp_bearing_rps_per_rad`
  (방위각 P 게인), `align_stable_samples`(허용치 안의 마커 표본이 연속
  이만큼 쌓여야 정렬 완료 — **시간이 아니라 표본 수**로 판정한다),
  `initial_align_confirm_s`(정렬 완료 후 contact로 넘어가기 전 유지 시간),
  `align_timeout_s`, `marker_control_timeout_s`/`marker_acquire_dropout_s`/
  `marker_release_s`(마커 신선도·안정성 판단), `z_ema_alpha`(거리 잡음
  완화용 지수이동평균).
- **접촉(contact)**: `marker_approach_speed_mps`→`marker_near_speed_mps`로
  마커 크기(`side_px`)가 `marker_blind_arm_side_px`에 가까워질수록 연속
  감속, `marker_blind_min_side_px`/`marker_growth_*`(마커가 화면 밖으로
  나가기 직전 "크기 증가 추세"를 확인해 블라인드 접촉을 무장), `contact_speed_mps`
  (블라인드 구간 속도, ★ 아래 전류 기준과 짝), `contact_current_threshold_a`
  (★ **1.7A, 2026-09-01 실측 확정** — `contact_speed_mps=0.10`에서 잰 값이라
  속도를 바꾸면 다시 재야 한다), `contact_current_hold_s`(전류가 기준
  이상으로 이 시간 연속 유지돼야 접촉 확정, 기본 0.5초), `contact_visual_timeout_s`
  /`blind_contact_distance_m`/`contact_timeout_s`(폭주 방지 상한).
- **잠금/검증**: `solenoid_timeout_s`, `verify_pull_speed_mps`(후진 당김
  속도), `verify_current_threshold_a`(0이면 `contact_current_threshold_a`를
  그대로 씀), `verify_current_confirm_samples`, `verify_timeout_s`.

**주의 — 선언됐지만 현재 미사용인 키**: `k_lateral`/`ki_lateral`/
`kd_lateral`/`k_yaw`/`ki_yaw`/`lateral_integral_limit_m_s`/
`yaw_integral_limit_deg_s`/`k_roll`/`k_pitch`/`lateral_tolerance_m`/
`lateral_tolerance_marker_ratio`/`roll_tolerance_deg`/`pitch_tolerance_deg`/
`yaw_tolerance_deg`는 코드에 `declare_parameter`만 돼 있고, 2026-09-01
재작성된 정렬 로직(`_alignment_command`, 방위각 P 제어만 씀)에서는
어디서도 읽히지 않는다. `mission/coupling/status`의 `pi.lateral_integral_ratio_s`
/`pi.lateral_rate`/`pi.yaw_integral_deg_s`도 같은 이유로 값이 계속 `0`으로
발행되는 죽은 필드다. `coupling.yaml`에는 이 값들에 대한 상세한 실측
근거 주석이 남아 있어 실제로 쓰이는 값처럼 보이기 쉬우니, 정렬 튜닝은
반드시 위 "정렬" 항목의 실사용 키(`bearing_tolerance_deg`/
`kp_bearing_rps_per_rad`)만 건드릴 것 — 이전 PID 방식의 잔재로 보이며
정리 여부는 [TODO.md](../../TODO.md)를 확인한다.

## 토픽

- 구독: `marker/status` (String JSON — `x_mm`/`y_mm`/`z_mm`/`roll_deg`/
  `pitch_deg`/`yaw_deg`/`offset_x_px`/`side_px`/`raw_detected` 등),
  `motor/status` (String JSON — 전류/CAN 상태), `coupling/cmd` (String
  JSON — start/cancel/unlock/recapture), `solenoid_status` (Bool),
  `safety/stop` (Bool), `safety/emergency_stop` (Bool)
- 발행: `cmd_vel/coupling` (Twist, arbiter의 SOURCES 중 하나),
  `solenoid_cmd` (Bool), `marker/enable` (Bool — 블라인드 접촉 진입 시
  끄고 재시도 시 다시 켬), `mission/coupling/status` (String JSON —
  `active`/`start_pending`/`phase`/`eligible`/`eligibility_reason`/`reason`/
  `marker_visible`(=`raw_detected`, 화면 표시용)/`marker_filtered_visible`/
  `marker_raw_visible`/`marker_stable`/`errors`(`bearing_deg` 포함)/
  `contact_blind`/`marker_growth`/`marker_growth_armed`/
  `blind_contact_distance_m`/`current_a`/`current_threshold_a`/
  `contact_current_above_for_s`/`verify_current_threshold_a`/
  `verify_current_hits`/`solenoid_locked`/`linear`/`angular` 등 — 위 "주의"
  참고, `pi` 블록은 죽은 필드)

**참고**: 웹 콘솔도 `solenoid_cmd`에 직접 발행할 수 있다(수동 조작,
[console 문서](../ui/console.md) 참고) — `coupling.py`는 `locking` 단계
외엔 이 토픽에 발행하지 않으므로 평소엔 안 부딪히고, 자동 결합이
`locking`으로 들어간 순간엔 이 노드가 20Hz로 계속 재확인해 자동이
우선한다.

## 동작 요약 (phase 상태기계)

`ready` → (`coupling/cmd`로 `start`, `_eligibility()` 통과) → `align` →
(방위각이 `bearing_tolerance_deg` 안에 든 새 마커 표본이
`align_stable_samples`회 연속 + `initial_align_confirm_s` 유지) → `contact`
→ (접촉 전류가 `contact_current_threshold_a` 이상으로 `contact_current_hold_s`
연속) → `locking` → (`solenoid_status=true` 확인) → `verify_pull` →
(후진 당김 중 전류가 검증 기준을 `verify_current_confirm_samples`회 연속)
→ `locked`.

- `_eligibility()`는 활성 결합 없음(`active`), 미잠금(`locked` 아님),
  `contact_threshold() > 0`(자기 config 우선, 없으면 `motor/status`의
  공통값, 둘 다 0이면 진입 거부 — 실측 전 안전장치), `marker/status`
  신선(`marker_status_wait`), 카메라 프레임 신선(`camera_frame_wait`),
  마커 검출(`marker_wait`), CAN 연결(`can_unavailable`), **솔레노이드가
  이미 잠겨 있음**(`solenoid_unlocked` — 대기 중 걸쇠는 잠긴 게 정상이고
  접촉 후 `locking`에서 다시 잠금 명령으로 확정한다), 안전정지 아님
  (`safety_stop`)을 모두 통과해야 한다.
- `align`: 새 마커 표본이 올 때마다만(20Hz tick보다 느린 11Hz 마커 주기에
  맞춰) 방위각 P 제어를 다시 계산한다 — 안 그러면 같은 값에 두 번 반응해
  진동한다. 정렬 완료 확인 중에도 자세는 계속 유지하되 아직 전진하지
  않는다. 마커가 짧게(≤`marker_acquire_dropout_s`) 끊기는 건 무시하고,
  오래 끊기면 `marker_status_wait`/`marker_reacquiring`으로 대기하며
  기존 `start` 요청은 유지한다(실패 처리하지 않음). `align_timeout_s`를
  넘으면 `_schedule_marker_retry`로 `marker_wait`로 물러나 마커가 다시
  보이면 자동 재시도한다.
- `contact`: 마커가 보이는 동안은 방위각 P로 중심을 유지하며
  `marker_approach_speed_mps`→`marker_near_speed_mps`로 연속 감속
  접근한다. 마커가 원본 프레임에서 실제로 사라지면(다중 프레임 소실
  확정은 `marker_vision`이 이미 끝낸 상태) 조건 없이 블라인드 직진으로
  전환한다 — "가까워져서 화면 밖으로 나갔다"고 보기 때문이다(이전에는
  크기 증가 추세를 추가로 요구했으나, 접근이 중간에 끊기면 추세가 안
  쌓여 영영 블라인드로 못 들어가는 막다른 길이 있어 제거했다). 접촉 전류
  판정(`contact_current_hold_s` 연속 유지)은 시야 추적 중이든 블라인드
  중이든 동일한 기준으로 본다 — 결합봉이 마커가 화면을 채우기 전에
  먼저 닿을 수 있기 때문(2026-09-01 실측 기하 확인). 블라인드 구간은
  `blind_contact_distance_m`(추정 이동 거리, 전류가 오르면 카운트 중단)와
  `contact_timeout_s`(절대 시간) 둘 다로 상한을 둔다. 두 상한을 넘거나
  `contact_visual_timeout_s`(시야 추적 상한)를 넘으면 `_schedule_marker_retry`
  로 물러난다 — **실패로 끝나지 않고 정지 후 마커 재검출을 자동으로
  기다린다**(재시작 버튼 불필요).
- `locking`: 솔레노이드 잠금 명령을 계속 재확인 발행하며 `solenoid_status`
  가 `true`가 되면 `verify_pull`로, `solenoid_timeout_s`를 넘으면
  `_abort('solenoid_timeout', unlock=True)`.
- `verify_pull`: `verify_pull_speed_mps`로 후진하며 실제 하중 전류가
  걸리는지 확인한다 — 솔레노이드 상태만으로는 성공 처리하지 않는다
  (ArUco가 블라인드 전환 시 꺼져 있어 시야로는 확인 못 함). 전류 확정
  전에 솔레노이드가 풀리거나(`solenoid_unlocked`), 전류가 끊기거나
  (`verify_current_lost`), `verify_timeout_s`(기본 1초, `-0.08 m/s`
  기준 최대 약 8cm만 후진)를 넘으면 `_abort(..., unlock=True)`로 해제 후
  실패 처리한다.
- `_schedule_marker_retry`는 `active`를 내리되 `start_pending`을 다시
  세워 마커가 돌아오면 자동으로 재시작한다 — 결합 시도가 실패로
  끝나는 게 아니라 "마커 재검출 대기"로 낮아진다.
- `_abort`는 항상 정지 명령을 내고, `unlock=True`(취소/타임아웃 종류)면
  솔레노이드도 해제한다.

## 연결

- 위: `base/marker_vision`(`marker/status` — `x_mm`/`z_mm` 방위각 계산,
  `raw_detected`), `base/motor`(`motor/status`), `base/solenoid`
  (`solenoid_status`), `drive/test_console`/`ui/console`(`coupling/cmd`
  발행 — start/cancel 버튼, 2026-09-01부터 준비 센서 미충족만으로는
  시작 버튼을 막지 않는다), `mission/cargo_load`(`coupling/cmd`의
  `start` — `couple` 액션 수신 시, 2026-08-31 추가).
- 아래: `drive/arbiter`가 `cmd_vel/coupling`을 구독하며, `coupling_status.active`
  또는 `phase in (locking, error)`이면 **모드보다 우선해서** 이 소스를
  채택한다(`_source()` 최상단 검사) — `locked`(성공)는 제외돼 있어, 결합
  성공 후 취소하기 전까지 영영 못 움직이는 걸 막는다. **잠금(`locked`)
  이후에는 `mission/coupled_drive`가 이어받아 1호기 속도를 복제한다**
  ([coupled_drive](coupled_drive.md) 참고, arbiter는 `coupled` 모드로
  별도 전환돼야 진입).
- `base/motor`의 `motor.yaml` `contact_current_threshold_a`(공통 폴백,
  여전히 `0.0`)를 `mission/cargo_load`와 공유하되, 이 노드 자신의
  `coupling.yaml` 값(1.7A, 확정)이 우선한다 — [HARDWARE.md](../../HARDWARE.md)
  "접촉 전류 기준값" 참고.

## 구현 상태

완성, **결합 쪽 접촉 전류 기준(1.7A) 확정됨(2026-09-01)** — `_eligibility()`
가 더 이상 `current_threshold_unset`으로 막지 않는다. 방위각 기반 정렬·
블라인드 접촉 전환·전류 검증까지 `src/mission/test/test_coupling_verification.py`
(18개 케이스)로 자동화 테스트했다. **실물 재검증 필요**: 이 재작성
자체(방위각 게인, 블라인드 전환 조건)와 솔레노이드 극성 재정정(4번,
[solenoid](../base/solenoid.md) 참고)이 겹쳐 있어, `align`→`contact`→
`locked`까지 실기로 끝까지 확인해야 한다.

## 알려진 이슈

- **정렬 게인 이원화(위 "설정" 주의 참고)**: `coupling.yaml`에 실측
  근거가 상세히 남아 있는 `k_lateral`/`ki_lateral`/`kd_lateral`/`k_yaw`/
  `ki_yaw` 등은 현재 코드 경로에서 미사용이다. 다음에 정렬을 다시
  튜닝할 사람이 이 값을 조정하고 왜 안 바뀌는지 헤맬 수 있어 남겨둔다.
- `verify_pull` 단계는 ArUco가 꺼져 있는 블라인드 접촉 이후에 오는
  경우가 많아, 잠금 성공 여부를 시야로 재확인할 방법이 없다 — 전류
  검증이 유일한 확인 수단이라는 점을 실물 시험 때 감안한다.
- **2026-08-28**: [marker_vision](../base/marker_vision.md)이 4색 HSV →
  ArUco로 전환되면서 바뀐 필드명에 맞게 재배선됐다(과거 작업, 정상
  종료). `src/mission/test/test_coupling_marker_pose.py`로 필드
  파싱·EMA 스무딩을 자동화 테스트했다.

## 검증 실패와 해제 (2026-09-03 변경)

`verify_pull`에서 후진하며 전류가 안 오르면 예전에는
`_abort(..., unlock=True)`로 빠져 **솔레노이드를 풀었다**. 지금은 풀지 않고
결합을 다시 돌린다(`_verify_retry`) — 검증 실패는 "덜 붙었다"는 뜻이지
"풀어야 한다"는 뜻이 아니고, 걸쇠는 기계적으로 잠기기 때문이다.
`verify_retry_limit`(기본 3회)를 넘으면 멈추되 그때도 해제하지 않는다.

해제는 `coupling/cmd`의 `unlock` 하나뿐이고, 시험 화면의 **결합 해제**
버튼(`/api/coupling/unlock`)에서만 온다. `cancel`은 진행 중인 결합만 멈추고
잠금은 유지한다 — 예전에는 둘이 같은 분기라 취소할 때마다 풀렸다.

## 결합 완료 이후 (2026-09-03 추가)

`locked`가 되면 `drive/arbiter`가 스스로 주행 모드를 `coupled`로 올린다
(수동 조종 중 `manual`/`rc`에서는 올리지 않는다). 대회 자동 주행 구간에서는
사람이 `drive/mode_cmd`를 쏠 수 없어, 예전에는 결합 후 그대로 멈춰 섰다
(TODO 22번).
