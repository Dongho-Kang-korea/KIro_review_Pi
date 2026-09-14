# 구현 현황

**최신화**: 2026-09-04 (결합 이동 입력을 `/fleet/cmd_vel/unit2` 로 교체, 강체 조향 복제 — 그 전 2026-09-03 결합 재작성·솔레노이드 극성·CPU 부하 완화)

`docs/`(노드별 상세)와 `HARDWARE.md`(배선/실측)보다 한 단계 위, 워크스페이스
전체를 훑는 개요다. 코드를 읽고 정리한 시점 기준이며, 실물 검증 여부는
각 노드 문서와 [TODO.md](../TODO.md)를 함께 봐야 정확하다.

## 패키지별 노드

| 패키지 | 노드(entry point) | 기본 launch 포함 | 비고 |
|---|---|---|---|
| base | motor | robot/manual/manual_drive/test_drive/web_manual | 완성. **2026-08-29 CAN 버스 장애·복구 이력** — 아래 "CAN 버스 장애 이력" 절 참고. **2026-09-01 어댑터 탈부착 자동 복구 추가** — 탈부착으로 소켓이 죽어도 노드가 죽지 않고 `can_reconnect_interval_s`마다 스스로 재연결한다([motor](base/motor.md) 참고) |
| base | camera | robot/manual(console=true)/test_drive | 완성, 더미 프레임 폴백. **2026-09-01 CPU 부하 완화** — `frame_rate` 30→15, `cv2.setNumThreads(1)`(영상 노드 다중 실행 시 코어 경쟁 완화, [camera](base/camera.md) 참고) |
| base | marker_vision | robot/test_drive/web_manual | 완성. ArUco ID 1 실물 인식 확인(실제 운용이 ID 1만 쓰므로 2·3 확인은 불필요로 정리). **pose(solvePnP) 캘리브레이션 확정·실측 검증 통과**(1280x720, 30/60cm 검증) — x/y/roll/pitch/yaw 안정적, z는 멀수록 잡음 증가(60cm서 약 7%). `coupling`/`follow_leader` 재배선 완료(자동화 테스트 통과). **웹 스트림 해상도 분리**(2026-08-29): 검출은 1280x720 그대로, `vision/image/compressed`만 `stream_width`/`height`(기본 640x360)로 축소 — 실물 확인 완료(라이브 토픽 실측: 원본 60061B → 축소 29371B). **3축 좌표계 오버레이**(`draw_axes`, `marker/axes_enable`)도 같이 추가, 기본 꺼짐. **2026-09-01 다중 프레임 검출 히스테리시스 + 저조도 CLAHE/equalize 폴백 추가**(자동화 테스트, [marker_vision](base/marker_vision.md) 참고) |
| base | calibrate_camera | **미포함**(수동 `ros2 run` 전용) | 완성(체커보드/ArUco 단일 마커 두 모드). 체커보드 방식으로 실제 캘리브레이션 완료·검증까지 마침 |
| base | imu | robot/web_manual | **장착됨**(I2C 0x4B 실측 확인), 영점 미교정. 아루코 기준 360도 회전 검증 도구(`imu_aruco_360_test`, 2026-09-01부터 base 패키지로 이동)로 5회 실측 완료 |
| base | uwb | robot/test_drive | **장착·활성화됨**(2026-08-29 실측 확인), 오프셋 정밀 보정 남음. **2026-09-01 시리얼 통신 견고화 + by-id 고정 포트 경로**([uwb](base/uwb.md) 참고) |
| base | solenoid | robot/test_drive/**web_manual**(항상) | 완성, 실측 배선 전제. **극성 실측 정정**(2026-08-28 하루 두 번, **2026-09-01 네 번째로 재정정** — 지금은 `active_low: false`) — 실물 재검증 필요. 웹 콘솔 수동 조작 경로 추가. **2026-08-29 장시간 통전 과열 발견 → 2026-08-30 기본값을 잠금(무통전)으로 수정**, **2026-09-01 자동 재잠금 타이머(`release_timeout_s`) 추가**(실물 재검증 필요, [TODO.md](../TODO.md) 9번, [solenoid](base/solenoid.md) 참고) |
| fleet | follower | robot, **web_manual**(`start_fleet`, 기본 true) | 완성 |
| fleet | fake_leader | **web_manual**(`start_fake_leader`, 기본 true) | 완성, 벤치 시험 전용(robot.launch.py엔 없음). 1호기 없이 `/fleet/state`를 대신 발행해 추종을 단독 시험할 수 있게 함 — 실물 확인: `fleet_link`/`follow_available`이 1호기 없이 true, 모드 전환 API가 `follow`를 실제로 받아줌(2026-08-29) |
| mission | follow_leader | robot, **web_manual**(`start_fleet`) | 완성, UWB 없으면 마커 거리 폴백(pose 재배선 완료, 실제 동작). **2026-09-01부터 `mission/winter` 패키지로 이동**(겨울 제설 전용으로 재분류, entry point는 그대로 `follow_leader`) |
| mission | coupling | robot/test_drive | 완성. **2026-09-01 방위각(atan2) 기반 P 제어로 재작성**(7월 `docking_ctrl.py` 구조로 회귀) + 블라인드 접촉 진입 조건 재설계. **접촉 전류 기준(1.7A) 실측 확정** — 더 이상 `align` 진입을 막지 않는다(자동화 테스트 18개, [coupling](mission/coupling.md) 참고). 실물 재검증 필요 |
| mission | coupled_drive | robot/test_drive | **신규(2026-09-01)**. 결합(`locked`) 완료 후 1호기 속도를 그대로 복제해 함께 이동하는 협조 주행 노드 — 결합 로드가 거리를 물리적으로 고정하므로 거리 제어(추종)가 아니라 속도 복제를 쓴다. 입력은 1호기가 50 Hz로 내는 `/fleet/cmd_vel/unit2`(Twist)다 — 2026-09-04 이전에는 1호기가 실은 적 없는 `/fleet/state.leader_linear`를 보고 있어 배선이 아예 안 붙어 있었다. 결합부가 강체라 조향도 복제한다. 단위 테스트만 있고 실물 미검증([coupled_drive](mission/coupled_drive.md) 참고) |
| mission | cargo_load | robot, **web_manual**(항상) | 완성(코드), 실물 검증은 3단계 하드웨어 필요. 화물 잠금은 서보 없이 `hold` 액션에서 바로 `loaded`로 완료 처리(서보 미사용 결정, 2026-08-30). 웹 콘솔 수동 컨베이어 조작(`roller_manual_cmd`) 추가·자동화 테스트 완료. **2026-09-01 적색 검출 CPU 부하 완화**(`red_detect_rate_hz` 5Hz로 스로틀, `cv2.setNumThreads(1)`) |
| drive | arbiter | robot/manual/manual_drive/test_drive/web_manual | 완성. **2026-09-01 `coupled` 모드 추가**(결합 이후 `cmd_vel/coupled` 소스, `follow`와 같은 `/fleet/state` 두절 강등 규칙, [arbiter](drive/arbiter.md) 참고) |
| drive | manual | (launch 미포함, `ros2 run`/manual_drive) | 완성, TTY 전용 |
| drive | test_console | manual(console=true)/test_drive | 완성. **2026-09-01 화면 갱신** — 마커 연결 표시를 원본 검출값 기준으로, 결합 시작 버튼 비활성 조건 완화([test_console](drive/test_console.md) 참고) |
| mars_console | console | web_manual, 또는 운용 PC에서 단독 실행 | 완성, 1·2호기 통합. `drive/status` 모드 동기화 버그 수정, 솔레노이드/컨베이어/ArUco 축 오버레이 수동 조작 추가(전부 자동화 테스트 완료), 상태 표에 pose 수치(x/y/z/roll/pitch/yaw) 표시 추가(2026-08-29) |

## 미장착·미보정 하드웨어 — 여름 적재 3단계

| 단계 | 필요한 것 | 상태 |
|---|---|---|
| `approach` | UWB | **장착·활성화됨**(2026-08-29), 오프셋 정밀 보정 남음 |
| `search` → `turning` | BNO086 | **장착됨**(I2C 0x4B), heading 영점 미교정 |
| `backing` → `contact` | 1호기 쪽 `contact_current_threshold_a` 실측값 | 미실측(0.0, 1호기 config) |

접촉 전류 기준은 미션마다 **따로** 잡는다(`base/config/motor.yaml`은
공통 폴백일 뿐). 자동 결합(`coupling`)은 `mission/config/coupling.yaml`의
값이 **1.7A로 확정**(2026-09-01)돼 접촉 판정에 들어간다. 여름 화물 적재
쪽 판정은 1호기가 자기 config로 하며 아직 미실측(0.0)이라 두 미션의
현재 상태가 서로 다르다 — 자세한 내용은 [HARDWARE.md](../HARDWARE.md)
"접촉 전류 기준값" 참고.

추종(`follow_leader`)은 UWB 없이도 `follow.yaml`의
`allow_marker_distance_fallback`이 켜져 있으면 마커 solvePnP 거리로 동작한다.

## CAN 버스 장애 이력 (2026-08-29)

`web_manual.launch.py` 재시작 중 CAN 통신이 완전히 끊긴 것을 발견해
진단·복구까지 마쳤다 — 나중에 비슷한 증상(모터 전부 무응답)이 재발하면
참고할 수 있게 경과를 남긴다.

1. **증상**: `motor/status`에서 좌/우 구동 모터(ID 1/2)·독립 롤러(ID 3)
   **셋 다** `axis_state`/`axis_error`/`current_a`가 전부 `null`,
   `can_connected: false`.
2. **원인 규명**(`journalctl -k`): `can0`는 MCP2515가 아니라
   **PEAK-System PCAN-USB 어댑터**다. 이 USB 어댑터가 특정 시점에
   5번 연속 탈부착(`can0 removed`/`attached`)됐다 — USB 접촉 불량이나
   순간 전원 단절로 추정. `can0-start.service`는 부팅 1회만 도는
   oneshot이라 그 이후로는 아무도 인터페이스를 다시 안 올려서 계속
   `DOWN`으로 남아 있었다.
3. **1차 조치**(`sudo ip link set can0 up type can bitrate 1000000
   restart-ms 100`)로 인터페이스 자체는 살아났지만(`state ERROR-ACTIVE`),
   **RX가 계속 0**이었다 — Pi→버스 송신(TX)은 정상인데 컨트롤러 응답이
   전혀 없어, 이 시점엔 컨트롤러 전원/배선 문제로 좁혀 보고 있었다.
4. **최종 해결**: 로봇 전원을 완전히 껐다 켜니(사용자 조치) CAN 버스가
   완전히 정상화됐다 — `can0` RX 트래픽 정상 수신, `motor/status`에서
   좌/우 `axis_state=8`(CLOSED_LOOP_CONTROL)·`axis_error=0`, 롤러도
   `axis_state=1`(IDLE, 정상 — 명령 대기 상태)·`axis_error=0`으로 응답.
   **결론: USB 어댑터 탈부착과 컨트롤러 무응답이 같은 전원 이상 사건의
   증상이었고, 전원 재인가로 함께 해결됐다.**
5. **재발 및 근본 조치**(같은 날 오전, 다른 세션
   `agent/follow-yaw-control-20260829`에서 확인, 이 브랜치로 반영):
   `candump can0`가 "Network is down"으로 실패하는 증상이 다시 발생.
   `journalctl`로 확인하니 `peak_usb ... Rx urb aborted (-71)` →
   `can0 removed` → 재부착 이벤트가 있었고, 위 2번에서 지적한
   `can0-start.service`의 한계(부팅 1회만 실행)가 그대로 재현된
   것이었다 — USB가 순간적으로 재열거되면 그 뒤로 아무도 인터페이스를
   다시 올려주지 않아 계속 `DOWN`으로 남는다. **udev 규칙
   `/etc/udev/rules.d/98-can0-autostart.rules` 추가로 근본 해결**:
   `can0` net 디바이스에 `ACTION=="add"` 이벤트가 오면(USB 재열거로
   인터페이스가 다시 생성될 때마다) `systemctl --no-block restart
   can0-start.service`를 실행해 자동으로 `ip link set can0 up`을
   재적용한다. `udevadm trigger --action=add --subsystem-match=net
   /sys/class/net/can0`로 재열거를 흉내 내 서비스가 실제로 재시작되고
   `can0`이 다시 `UP`/`ERROR-ACTIVE`로 올라오는 것을 확인했다. 이 udev
   규칙 파일은 시스템 설정(`/etc/udev/rules.d/`)이라 이 git 저장소에는
   들어 있지 않다 — 로봇을 새 이미지로 재설치할 때는 위 규칙을 다시
   만들어 줘야 한다. **주의**: 이 udev 규칙은 "인터페이스가 DOWN으로
   남는 문제"만 고친다 — [TODO.md](../TODO.md) 10번의 "컨트롤러
   자체가 무응답"(RX 0, 배선/전원계 원인 추정) 재발과는 다른 증상이니
   혼동하지 않는다.

남은 확인: 독립 롤러 실제 물리 회전([TODO.md](../TODO.md) 7번 —
명령을 줬을 때 `axis_state`가 `8`로 바뀌고 실제로 도는지), CAN
컨트롤러 무응답 재발의 전원계 원인([TODO.md](../TODO.md) 10번).

## 배선/네임스페이스 전제

- **3호기 지원 제거(2026-08-31, TODO 19 완료)**: `robot.launch.py`/
  `manual.launch.py`의 `unit` 인자가 2만 받고, `console.py`의 `UNITS`
  표도 2호기만 만든다. 1호기 Fleet Protocol v1.2 가 "여름 적재는
  1·2호기 둘이서 한다"로 3호기를 이미 뺀 상태였고(1호기 쪽 커밋에서
  `unit3` 토픽·코드 삭제 완료), 사용자가 삭제를 확정해 이쪽도 맞췄다.
  호기를 다시 늘리려면 그 두 곳의 숫자만 되돌리면 된다.
- 모든 노드가 `/unit2` 아래에서 뜬다(1호기와 토픽명 충돌
  방지). 코드 내부 토픽은 전부 상대 이름 — 절대 경로는 `/fleet/*` 규약
  토픽 4개뿐(`state`/`cargo/target`/`cmd/unitN`/`status/unitN`).
- `ROS_DOMAIN_ID=10`이 실전 기본값. `manual_drive.launch.py`/
  `web_manual.launch.py`는 시험 격리를 위해 `test_domain`(기본 77)으로
  덮어쓴다.

## 이 문서의 갱신 규칙

코드 구현 상태 자체(스텁→구현, 새 노드 추가 등)가 바뀌면 이 표를 같이
고친다 — [AGENTS.md](../AGENTS.md) 2절 참고.
