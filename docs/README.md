# docs/ — 모듈·노드별 상세 문서

**최신화**: 2026-09-03

최상위 문서([README.md](../README.md), [HARDWARE.md](../HARDWARE.md),
[RUNBOOK.md](../RUNBOOK.md), [TODO.md](../TODO.md))는 워크스페이스
전체를 보는 개요다. 이 폴더는 그 아래 단계 — **노드 하나하나가 정확히
무엇을 하고, 무엇을 구독/발행하고, 위아래로 무엇과 연결되는지**를 다룬다.
사람(인수인계자)과 에이전트 양쪽이 참고 대상이다.

## 이 문서들을 최신 상태로 유지하는 규칙

코드를 고치면 관련 문서도 같이 고친다는 저장소 공통 정책은
[AGENTS.md](../AGENTS.md)에 있다 — 이 폴더도 그 정책의 적용 대상이다.
**TODO.md 항목의 생애주기**(제안 시 TODO 등록 → 구현 → 테스트 → 완료 후
삭제)도 AGENTS.md에 정리돼 있으니, 새 작업을 시작하기 전에는 그쪽을 먼저
본다.

## 층 순서대로

| 층 | 폴더 | 문서 |
|---|---|---|
| 1 하드웨어 | [base/](base/) | [motor](base/motor.md) · [camera](base/camera.md) · [marker_vision](base/marker_vision.md)(ArUco 마커 인식, pose는 옵션) · [calibrate_camera](base/calibrate_camera.md)(커미셔닝 도구, launch 미포함) · [imu](base/imu.md) · [imu_aruco_360_test](base/imu_aruco_360_test.md)(IMU 영점 검증 도구, launch 미포함, 2026-09-01 이 패키지로 이동) · [uwb](base/uwb.md) · [solenoid](base/solenoid.md) |
| 2 함대 통신 | [fleet/](fleet/) | [protocol](fleet/protocol.md)(공용 규약) · [follower](fleet/follower.md) · [fake_leader](fleet/fake_leader.md)(1호기 없이 벤치에서 추종을 시험하는 가짜 리더, web_manual.launch.py 전용) · [rc_bridge](fleet/rc_bridge.md)(1호기 경유 RC 조종기 채널 → cmd_vel/rc, 토픽/채널 매핑 임시값) |
| 3 미션(인식+제어) | [mission/](mission/) | [follow_leader](mission/follow_leader.md)(추종, 전 계절 — 패키지는 `mission.winter`) · [coupling](mission/coupling.md)(자동 결합) · [coupled_drive](mission/coupled_drive.md)(결합 이후 협조 주행, 2026-09-01 신규) · [cargo_load](mission/cargo_load.md)(여름 화물 적재) |
| 4 주행 중재 | [drive/](drive/) | [arbiter](drive/arbiter.md)(모드 소유 + 최종 속도 선택) · [manual](drive/manual.md)(키보드, TTY 전용) · [test_console](drive/test_console.md)(호기 로컬 시험 화면 :5000) |
| 5 운용 화면 | [ui/](ui/) | [console](ui/console.md)(mars_console, 운용 PC 전용 :5100) |
| — launch | [bringup/](bringup/) | [launch](bringup/launch.md)(mars_launch 패키지의 launch 파일 5종) |

정적 분석 기반 전체 구현 현황(스텁 목록, 테스트 커버리지)은
[IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md) — 이 폴더보다 개요에
가깝다.

## 각 노드 문서의 형식

역할 → 실행(어느 launch) → 설정(어느 yaml) → 토픽(구독/발행) → 연결
(위아래 노드) → 구현 상태(완성/스텁/부분, 실측 여부) → 알려진 이슈(TODO
링크). 숫자 값(게인·임계값 등)은 여기 옮겨 적지 않는다 — yaml이 계속
최신 출처이고, 옮겨 적으면 반드시 stale해진다.

## 이 워크스페이스의 네임스페이스 — 문서 읽기 전 알아둘 것

아래 모든 문서의 토픽 이름은 **노드 코드 안에서 쓰이는 상대 이름**이다.
실제로는 launch가 `/unit2` 네임스페이스를 씌운다
(`cmd_vel` → `/unit2/cmd_vel`). 예외는 호기 간 규약 토픽
(`/fleet/state`, `/fleet/cargo/target`, `/fleet/cmd/unitN`,
`/fleet/status/unitN`) 뿐이며, 이 넷은 코드에도 절대 경로로 박혀 있다.
