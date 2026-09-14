# marker_vision — `src/base/base/marker_vision.py` + `aruco_tracker.py`

**최신화**: 2026-09-03

## 역할

카메라 영상에서 ArUco 마커(`DICT_4X4_50`, 허용 ID 1/2/3 — 실제로 쓰는 것은 1호기용 ID 1 하나이고 나머지는 예비다. 각 ID 가 어느 호기를
가리킨다)를 찾아 픽셀 좌표(중심, 오프셋, 변 길이, 이미지 평면 회전각)로
발행한다. UI와 완전히 분리된 순수 인식 노드.

**2026-08-28에 4색 HSV 마커(`ColorMarkerTracker`) → ArUco(`ArucoMarkerTracker`)로
전환했다** (`ros2_ws_huisu`에서 이식). `aruco_tracker.py`가 검출 로직을
담당하고, `marker_vision.py`는 그 위에서 ROS 파라미터/토픽만 얹는다.

## 실행

`mars_launch/robot.launch.py`, `test_drive.launch.py`에서 뜬다.

## 설정

`src/base/config/marker.yaml` — 딕셔너리(`dictionary`), 허용 ID
(`allowed_ids_json`), 목표 ID(`target_id`, -1이면 화면에서 가장 큰 허용
마커 자동 선택 — **2026-08-31부터 기본값을 `1`로 방어적 고정**했다.
1호기 뒤 ArUco가 ID 1로 확정됐고 지금은 물리 마커가 1개뿐이라 `-1`도
안전하지만(TODO.md 12번 결론), 다른 호기용 마커가 실제로 추가되면 `-1`은
화면에 순간 더 크게 잡히는 엉뚱한 마커를 따라갈 위험이 있어 미리 고정해
뒀다 — TODO.md 17번), 마커 실측 크기(`marker_size_mm`, 검은 패턴 기준
40x40mm — pose 계산에 실제로 쓰인다), `plate_size_mm`(흰 배경판 크기,
표시용 메타데이터), `border_bits`, `min_side_px`, `jpeg_quality`,
`detection_timeout_s`.

**다중 프레임 검출 히스테리시스** (2026-09-01 추가, [test_marker_detection_hold.py](../../src/base/test/test_marker_detection_hold.py)로 검증) —
원시(raw) 검출은 30fps로 계속 흔들릴 수 있어, `decision_window_s`
(기본 0.35초) 슬라이딩 윈도우로 최근 표본을 모아 히스테리시스를 건다.
검출 확정(`acquire_*`)은 쉽게(표본 `acquire_min_hits`=4개 이상 +
성공률 `acquire_hit_ratio`=60% 이상), 소실 확정(`lost_*`)은 어렵게
(표본 `lost_min_samples`=6개 이상 + 실패율 `lost_miss_ratio`=80%
이상 + 마지막 성공 이후 `lost_confirm_s`=0.25초 경과) 잡아, 한두 프레임
누락이나 성공/실패가 섞이는 상황을 소실로 처리하지 않는다.
`status_rate_hz`(기본 20Hz — 예전엔 고정 5Hz였다)로 상태 발행 주기를
조정할 수 있다.

**저조도 검출 폴백** (2026-09-01 추가, `aruco_tracker.py`) —
기본 검출이 실패한 프레임만 `clahe_fallback`(기본 `true`, 국소 대비
보정 후 재시도) → 그래도 실패하면 `equalize_fallback`(기본 `true`,
전역 `equalizeHist` 후 재시도) 순으로 최대 2번 더 시도한다. 조명이
계속 나빠 기본 경로가 거의 항상 실패하는 현장에서는 폴백이 사실상
상시 경로가 되어 프레임마다 검출을 3번 돌아 CPU를 3배 쓰게 된다
(실측: 그런 조건에서 marker_vision 74%, 카메라 18fps, 검출률 0%) —
그럴 때는 `equalize_primary`를 `true`로 올려 평활화를 1차 경로로
승격하면 한 번만 돈다. 모듈 임포트 시 `cv2.setNumThreads(1)`도
호출한다([camera](camera.md) "영상 노드 CPU 부하" 참고).

**pose 계산 설정** (2026-08-28 추가) — `camera_matrix_json`/
`dist_coeffs_json`이 비어 있으면(기본값) pose를 계산하지 않는다.
[calibrate_camera](calibrate_camera.md)의 결과를 검토해 이 두 값과
`calibration_width`/`calibration_height`를 채우면 pose가 켜진다.
현재 실행 중인 카메라 해상도가 `calibration_width`/`height`와 다르면
자동으로 pose를 끈다(다른 해상도의 `camera_matrix`는 안 맞기 때문).

**표시(웹 스트림) 해상도 분리** (2026-08-29 추가) — `stream_width`/
`stream_height`(기본 640/360)를 설정하면 `vision/image/compressed`로
내보내는 **주석 프레임만** 이 크기로 줄여 인코드한다. 검출/pose 계산은
항상 원본(`camera.yaml`의 `width`/`height`, 현재 1280x720) 프레임에서
먼저 끝난 뒤 리사이즈하므로 정확도에는 영향이 없다 — `mars_console`
웹 화면까지 나가는 JPEG 용량만 줄여 딜레이를 줄이려는 목적이다. `0`으로
두면(둘 다 0이어야 함) 리사이즈 없이 원본 해상도 그대로 나간다.

**3축 좌표계 오버레이** (2026-08-29 추가) — `draw_axes`(기본 `false`)를
켜면 선택된(target) 마커 위에 `cv2.drawFrameAxes`로 solvePnP 결과의
X/Y/Z 축을 그린다(길이는 `axes_length_mm`, 기본 30mm). pose가 꺼져
있거나(캘리브레이션 없음/해상도 불일치) 마커가 선택되지 않으면 그리지
않는다. 실행 중에는 `marker/axes_enable`(Bool)로 켜고 끌 수 있다 —
`mars_console` 웹 화면의 "3축 좌표계 표시" 토글 버튼이 이걸 발행한다.
기본을 꺼둔 이유는 실시간성 우선 — 필요할 때만 켜서 눈으로 pose 값을
확인하고 다시 끄는 용도다.

## 토픽

- 구독: `camera/image/compressed`, `marker/enable` (Bool),
  `marker/command` (String JSON — `{"action":"enable"|"disable"|"capture"}`,
  `capture`는 ArUco에서 무의미해 경고만 남기고 무시), `marker/axes_enable`
  (Bool — 3축 좌표계 오버레이 on/off, 2026-08-29 추가)
- 발행: `marker/status` (String JSON — `detected`/`detected_ids`/
  `markers`(ID별 픽셀 중심·오프셋·변 길이·회전각, pose가 켜져 있으면
  마커별로 `x_mm`/`y_mm`/`z_mm`/`roll_deg`/`pitch_deg`/`yaw_deg`/`rvec`도
  포함)/`pixel_only: true`/`pose_available`(동적 — calibration 설정
  여부와 해상도 일치 여부로 결정)/`axes_enabled`(현재 3축 오버레이
  on/off, 2026-08-29 추가)/`raw_detected`(**2026-09-01 추가** — 위
  히스테리시스를 거치지 않은 원시 프레임 검출값. `detected`는 필터를
  거친 값이라 짧은 소실 동안 마지막 검출을 유지할 수 있어, 화면의
  "연결됨" 표시처럼 즉시 반응해야 하는 곳은 `raw_detected`를 우선
  본다 — [test_console](../drive/test_console.md) 참고)/
  `detection_age_s`/`detection_window_samples`/`detection_window_hits`/
  `detection_hit_ratio`/`detection_miss_ratio`/`lost_confirm_s`
  (2026-09-01 추가, 히스테리시스 진단용)), `marker/pose` (PoseStamped, 단위 m —
  **pose가 켜져 있고 마커가 검출됐을 때만** 발행. pose가 꺼져 있으면
  이 토픽 자체가 안 나온다), `vision/image/compressed` (CompressedImage,
  검출 박스·ID가 그려진 주석 영상 — `stream_width`/`height`가 설정돼
  있으면 그 크기로, 아니면 원본(1280x720) 그대로)

## 동작 요약

- `camera/image/compressed`을 받을 때마다 `ArucoMarkerTracker.process()`가
  프레임을 디코드 → 그레이스케일 → `cv2.aruco` 검출 → 허용 ID/최소 변
  길이(`min_side_px`) 필터 → (pose 켜져 있으면) `cv2.solvePnP`
  (`SOLVEPNP_IPPE_SQUARE`)로 마커별 x/y/z/roll/pitch/yaw 계산 → 가장
  큰(또는 `target_id` 지정) 마커 선택 → (`stream_width`/`height` 설정
  시) 주석 영상만 리사이즈 → 인코드 순으로 처리한다. 리사이즈는 검출/pose
  계산이 모두 끝난 뒤에 일어나므로 정확도와 무관하다.
- pose 좌표계: 카메라가 원점, marker_vision이 물체점을 TL/TR/BR/BL 순서로
  주는 OpenCV 표준 convention(마커 평면 기준 X 오른쪽/Y 위/Z 카메라 쪽)을
  쓴다. `roll_deg`/`pitch_deg`/`yaw_deg`는 회전행렬에서 뽑은 Tait-Bryan
  각이고, `rvec`는 원본 축각(axis-angle) 회전 벡터 그대로다.
- `detection_timeout_s` 안에 새로 검출되지 않으면 `marker/status.detected`가
  `false`로 떨어진다(안전 타임아웃).
- `enabled=false`나 `{"action":"disable"}`을 받으면 결과를 즉시 빈 값으로
  비운다.

## 연결

- 위: `base/camera`(`camera/image/compressed`) — **주의**: `camera.yaml`의
  `image_topic`이 예전에 절대 경로(`/camera/image/compressed`)로 돼 있어
  `/unit2` 네임스페이스 밖으로 발행되던 버그가 있었다. 2026-08-28에 상대
  경로로 고쳤다.
- 아래: `mission/follow_leader`(`marker/pose` 기대 — pose가 꺼져 있는 한
  안 옴, 폴백 비활성 상태), `mission/coupling`(`marker/status`의
  `x_mm`/`y_mm`/`z_mm`/방위각·`raw_detected`를 소비 — 2026-08-28에
  새 필드명으로 재배선 완료, 2026-09-01 재작성에서도 그대로 씀 —
  [coupling](../mission/coupling.md) 참고), `drive/test_console`/`ui/console`
  (표시용 — `mars_console`은 2026-08-28부터 `vision/image/compressed`를
  기본 화면으로 보여준다).

## 구현 상태

**완성, pose 캘리브레이션 확정·실측 검증 통과(2026-08-28)**. 실물
카메라로 ID 1 인식 확인. 실제 운용은 ID 1만 쓰므로(다른 호기 마커 없음)
2/3 실물 확인은 필요 없음으로 정리했다.

pose 캘리브레이션은 시행착오를 거쳤다 — ArUco 단일 마커 방식(점 4개)은
재투영 오차가 낮아도(0.154px) 30cm 실측 검증에서 50% 오차가 나와 폐기.
카메라를 640x360 → **1280x720**으로 올리고 노트북 화면에 체커보드를
띄우는 방식(프린터 없이)으로 재캘리브레이션(20장, 재투영 오차
0.022px)해 `marker.yaml`에 반영, 30cm·60cm 실측 검증 통과. 정확도
프로파일: x/y/roll/pitch/yaw는 안정적(표준편차 수 mm~5도, roll은 원형
통계 기준), **z(거리)는 멀수록 잡음이 커진다**(30cm 표준편차 1mm대,
60cm 표준편차 42mm/약 7%) — 마커가 화면에서 차지하는 픽셀이 줄어드는
자연스러운 현상.

## 알려진 이슈

없음. **2026-08-28**: [coupling](../mission/coupling.md)/
[follow_leader](../mission/follow_leader.md)가 새 필드명(`x_mm`/`y_mm`/
`z_mm`/`roll_deg`/`pitch_deg`/`yaw_deg`)에 맞게 재배선됐고 자동화
테스트로 확인했다(`src/mission/test/`).
