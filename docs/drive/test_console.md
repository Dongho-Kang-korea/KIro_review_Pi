# test_console — `src/drive/drive/test_console.py`

**최신화**: 2026-09-03

## 역할

이 호기 위에서 도는 정비용 웹 화면(Flask). 카메라·상태 확인과 비상 정지,
자동 결합 시작/취소 버튼만 제공한다 — **주행 명령은 내지 않는다**. 운용
전체를 다루는 `mars_console`(운용 PC용, :5100)과는 별개.

## 실행

`mars_launch/test_drive.launch.py`(항상), `manual.launch.py`
(`console:=true`일 때만)에서 뜬다. 기본 포트 5000.

## 설정

`src/drive/config/arbiter.yaml`을 그대로 재사용(파라미터 선언용, 실제로
쓰는 값은 `host`/`port`/`unit`뿐). `unit`
파라미터로 규약 상태 토픽(`/fleet/status/unit{unit}`)만 갈라진다.

## 토픽

- 구독: `camera/image/compressed`, `drive/status`,
  `/fleet/status/unit{unit}` (절대경로), `motor/status`, `marker/status`,
  `mission/cargo/status`, `imu/status`, `mission/coupling/status`,
  `uwb/status`, `solenoid_status`
- 발행: `safety/emergency_stop` (Bool), `coupling/cmd` (String JSON —
  start/cancel)

## 동작 요약

- HTTP: `/`(화면), `/api/status`(전체 상태 스냅샷), `/api/emergency-stop`,
  `/api/reset-emergency`, `/api/coupling/start`, `/api/coupling/cancel`,
  `/camera.mjpg`(MJPEG 릴레이).
- 상태는 스레드락(`self.lock`)으로 보호된 딕셔너리에 모아 두고
  `/api/status`가 스냅샷을 JSON으로 반환한다.
- **2026-09-01 화면(`templates/test_console.html`) 변경**: 마커 "연결됨"
  표시를 `marker.detected`(필터를 거쳐 짧은 소실 동안 유지될 수 있음)
  대신 `marker.raw_detected`(원본 프레임 검출값, [marker_vision](../base/marker_vision.md)
  참고)로 바꿔 즉시 반응하게 했다. 결합 시작 버튼(`b-start`)은 예전엔
  `coupling/status.eligible`이 아니면 무조건 막았는데, `coupling.py`가
  `start`를 `start_pending`으로 보관해 조건이 갖춰지면 알아서 시작하는
  구조로 바뀌면서(위 [coupling](../mission/coupling.md) 참고) 버튼도
  진행 중(`active`)이거나 이미 결합됨(`phase==locked`)이거나 안전정지
  중일 때만 막도록 완화했다 — 준비 센서가 아직 없어도 미리 눌러 두면
  된다. 이유 표시 사전(`reason`)에 `solenoid_unlocked`(솔레노이드
  잠금 대기)도 추가했다.

## 연결

- 위: 사람(운용자, 브라우저), `base/camera`, `drive/arbiter`, `fleet/follower`,
  `base/motor`, `base/marker_vision`, `mission/cargo_load`, `base/imu`,
  `mission/coupling`, `base/uwb`, `base/solenoid`.
- 아래: `drive/arbiter`(`safety/emergency_stop`), `mission/coupling`
  (`coupling/cmd`).

## 구현 상태

완성.

## 알려진 이슈

없음 — 발견되면 [TODO.md](../../TODO.md)에 등록한다.

## 결합 해제 키 (2026-09-03 분리)

`결합 취소`(`/api/coupling/cancel`)와 `결합 해제`(`/api/coupling/unlock`)를
다른 버튼으로 나눴다. 예전에는 버튼 하나가 둘을 겸해서, 진행 중인 결합을
취소할 때마다 솔레노이드까지 풀렸다.

- **결합 취소**: 진행 중인 결합만 멈춘다. 잠금은 그대로 유지된다.
- **결합 해제**: 솔레노이드를 푸는 **유일한 경로**다. 잠금은 기계적으로
  유지되고 해제는 사람이 누를 때만 한다는 확정 사항(2026-09-03)에 따라,
  자동으로는 어디서도 불리지 않는다. 실수 방지를 위해 확인 창을 띄운다.
