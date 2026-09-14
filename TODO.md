# TODO

**최신화**: 2026-09-03 (21번 구현·자동화 테스트 완료, 실물 검증 대기)

이 문서는 "필요성은 있는데 아직 구현 안 한 것"만 담는다. 드나드는 규칙
전체는 [AGENTS.md](AGENTS.md) 3절 "TODO 생애주기"를 따른다 — 요약하면:
새 항목은 구현 전에 여기 먼저 등록하고, 구현했다고 바로 지우지 않고
**자동화 테스트 또는 [RUNBOOK.md](RUNBOOK.md) 절차로 실물 검증까지 끝나야**
지운다. 검증 전이면 "구현됨 — 테스트 필요"로 표시만 바꾼다.

각 항목은 **필요성 / 구현 위치 / 구현 방향 제안** 3단 형식을 따른다.

## 1. UWB(DWM1001) 실장착 (구현됨 — 정밀 오프셋 보정·미션 통합 테스트 필요)

- **필요성**: 여름 화물 적재의 `approach` 단계가 UWB 거리 없이는 진행되지
  않는다. 추종은 `allow_marker_distance_fallback`으로 UWB 없이도 되지만,
  정확도는 UWB가 붙어야 올라간다.
- **구현 위치**: `src/base/base/uwb.py`, `src/base/config/uwb.yaml`.
- **2026-08-29 실물 확인 경과**: 보유한 DWM1001 모듈 3개(`DW4823`,
  `DWD00C`, `DW559E`)를 시리얼 shell(`nmg`/`si`)로 직접 조회해보니 **셋
  다 공장 기본값인 앵커(anchor) 상태**였다 — 태그가 하나도 없어 거리
  측정 자체가 불가능했다. `DW4823`을 `nmt` 명령으로 태그(Tag Normal)로
  전환하고 `DWD00C`를 앵커로 둔 채 `les`/`lec`으로 실측 거리(`0.79~0.93m`)
  확인 완료 — 하드웨어 자체는 정상.
  - 이 과정에서 `uwb.py`의 진짜 버그 2개를 발견함(기존 코드는 모듈이
    "이미 스트리밍 중"이라고 잘못 가정하고 있었다):
    1. **능동 폴링 누락**: DWM1001은 명령 없이는 데이터를 스스로 안
       보낸다(직접 확인 — 5초 수동 대기해도 무응답). `_tick()`이
       `in_waiting`만 읽고 아무 명령도 안 보내 항상 빈 값만 나옴 —
       `lec\r`을 주기적으로 써 줘야 한다.
    2. **단위 버그**: `lec` 응답(`DIST,1,AN0,D00C,0.00,0.00,0.00,0.79`)의
       마지막 필드는 **미터**인데 `_parse()`가 이를 그대로
       `distance_mm`으로 발행 — 1000배 차이.
- **2026-08-29 후속 — 정지 감지로 인한 갱신 지연 발견 및 해결**: 위 두
  버그를 고친 뒤 실측하니 `distance_mm`이 한 번 갱신되고 몇 초~수십 초씩
  안 바뀌는 문제가 있었다. 태그 셸 `si`에 `stat_det=1`(정지 감지 켜짐),
  `aurg`에 `nom=1 stat=100`(정지 상태면 갱신 주기가 10초로 떨어짐)이 원인
  — 태그가 벤치에 가만히 있어 "정지" 상태로 인식된 것. 사용자 제공 명령으로
  해결:
  ```
  acts 0 0 0 1 0 1 1 2 0   # stnry_en=0으로 정지 감지 끔
  aurs 10 10               # normal/stationary 둘 다 1초(10×100ms)
  reset                    # 플래시 저장, 전원 재인가에도 유지
  ```
  `reset` 직후 모듈이 일시적으로 바이너리 응답만 돌려주며 셸이 안 살아난
  적이 있었는데 **USB 케이블을 뽑았다 다시 꽂으니 정상 복구**됐다(소프트웨어
  재시도로는 안 풀림 — 원인 불명, 재발하면 같은 방법으로 복구). 이후
  `aurg: nom=10 stat=10` 유지 확인, 거리값이 약 1~1.5초 간격으로 계속
  바뀌는 것 확인 완료.
- **검증 상태**: `ros2 run base uwb --ros-args -p enabled:=true -p
  port:=/dev/ttyACM0`로 실제 `distance_mm` 토픽이 살아있는 값(0.7~0.9m
  대)으로 발행되는 것을 ROS 토픽 레벨에서 확인했고, 사용자가 정면 기준
  거리를 육안으로 비교해 "정확도가 어느 정도 맞는 것 같다"고 확인함. 이를
  근거로 `uwb.yaml`을 `enabled: true`, `port: '/dev/ttyACM0'`로 정식
  반영함(`offset_mm`은 아직 `0.0` — 줄자 등 정확한 기준 거리로 잰 보정이
  아니라 육안 확인 수준이라 남겨둠).
- **아직 안 된 것**:
  1. 줄자 등 정확한 기준 거리와 비교한 `offset_mm` 정밀 보정.
  2. 여름 화물 적재 `approach` 단계에서 실제 미션 로직과 통합 시험(지금까지는
     드라이버 단독 확인만 됨).
  3. 앵커(`DWD00C`)는 USB 전원만 쓰면 뽑는 순간 꺼진다 — 실사용 시 배터리
     등 별도 전원 필요.
- **2026-09-01 시리얼 견고화 + 포트 경로 변경**: `port`를 `/dev/ttyACM0`
  에서 장치 고유 `/dev/serial/by-id/...` 경로로 바꿨다(재부팅 열거
  순서로 번호가 바뀌는 문제 대응). 셸 진입 명령 간격(`shell_enter_delay_s`)
  과 응답 두절 시 자동 재연결(`data_timeout_s`/`reconnect_interval_s`)도
  추가했다 — [docs/base/uwb.md](docs/base/uwb.md) 참고. **주의**: 위
  2026-08-29 기록에 남긴 "`reset` 직후 바이너리 TLV 모드에 갇히면
  소프트웨어 재시도로는 안 풀리고 USB 케이블을 직접 뽑았다 꽂아야
  풀렸다"는 실측 사례가 있다 — 이번 자동 재연결이 그 케이스까지
  실제로 복구하는지는 아직 실물로 확인 전이다.

## 2. 접촉 전류 기준값(`contact_current_threshold_a`) 실측 — **미션별 2개** (결합 쪽 구현됨 — 적재 쪽 실측 필요)

- **필요성**: 여름 화물 적재(`backing`→`contact`)와 자동 결합
  (`align`→`contact`)이 이 값을 공유한다. `0.0`인 동안은 두 미션 모두
  자동 접촉 판정에 진입하지 않는다(의도된 안전 상태이지 버그 아님).
- **2026-09-01 구조 변경**: 값을 **미션마다 따로** 갖게 나눴다. 실측에서
  접촉 전류가 접근 속도에 비례해 커지는 것이 확인됐기 때문이다 — 결합은
  빠르게 붙어 2A대, 적재는 느리게 밀어 1.2~1.4A대였다. 값 하나를 공유하면
  결합에 맞춘 값은 적재에서 영영 안 걸리고, 적재에 맞춘 값은 결합에서
  빈 주행에도 오탐한다.
  - `mission/config/coupling.yaml` -> `contact_current_threshold_a` (결합용)
  - `mission/config/cargo.yaml` -> `contact_current_threshold_a` (적재용)
  - `base/config/motor.yaml` 의 같은 키는 **공통 폴백**으로 남는다
    (미션 값이 0 일 때만 쓰인다).
- **2026-09-01 결합 쪽 실측 완료**: `coupling.yaml`의
  `contact_current_threshold_a`를 실제 결합 속도(`contact_speed_mps=0.10`)로
  붙여본 값을 근거로 `1.7`로 확정. 같은 작업에서 결합 로직 자체도
  `align`→`contact`(부딪힘) →`locking`(솔레노이드 작동)
  →`verify_pull`(후진 당김으로 실제 고정 확인) 4단계로 나누고,
  `verify_current_threshold_a`(0이면 위 값 재사용)로 잠금 여부 자체도
  전류로 검증하도록 확장했다 — **정정(2026-09-03)**: 이전 기록에 "근접
  구간 마커 소실에 UWB 거리 폴백을 추가했다"고 남겼는데 확인해보니
  틀렸다. `contact_uwb_threshold_mm`는 `cargo_load`(적재) 쪽 config에만
  있는 값이고, `coupling.py`/`coupling.yaml`은 UWB를 전혀 쓰지 않는다.
  근접 구간(마커가 화면 밖으로 나감) 문제는 대신 **블라인드 접촉**으로
  풀었다 — 마커 크기가 커지는 추세(`marker_growth_*`)를 확인한 뒤 마커
  없이 정해진 속도로 더 직진하다가 접촉 전류로 확정하는 방식이다. 정렬
  자체도 화면 중심비 대신 **방위각**(`atan2(x_mm, z_mm)`) 기반 P
  제어로 재작성됐다 — 자세한 내용은
  [coupling 문서](docs/mission/coupling.md) 참고. **이 재작성 전체가
  아직 실물 재검증 전이다.**
- **아직 안 된 것**: `cargo.yaml`(적재용) 값은 아직 `0.0`이다. 적재
  단계가 **실제로 쓰는 속도**(`back_speed_mps`)로 접근한 상태에서
  재야 한다 — 위 노트대로 다른 속도로 잰 값은 대표성이 없다. 측정은
  `test_basic/current_web.py`로 한다.
- **구현 위치**: `src/base/config/motor.yaml`(폴백),
  `src/mission/config/coupling.yaml`, `src/mission/config/cargo.yaml`.
- **구현 방향 제안**: 실물 접촉 시 `motor/status`의 `current_a`를 관찰해
  임계값을 정하고, [RUNBOOK.md](RUNBOOK.md) 안전 체크리스트대로 비상 정지가
  듣는지 먼저 확인한 뒤 값을 올린다.
- **2026-08-31 측정 도구 추가**: `test_basic/current_monitor.py`.
  `motor/status`의 `current_a`를 10Hz로 받아 터미널에 실시간 막대그래프와
  실행 전체의 최저(low)·최고(high)를 같이 띄운다. Ctrl+C로 멈추면 요약을
  찍는다. 빈 주행으로 low를, 밀어붙여서 high를 잰 뒤 그 사이 값을 고르면 된다.

  ```bash
  # 모터 전원을 넣은 뒤 이것 하나만 실행하면 CAN 확인 -> motor 기동 ->
  # 전류 수신 확인 -> 감시 화면까지 순서대로 진행된다
  bash test_basic/measure_contact_current.sh

  # 감시 화면만 따로 띄울 때
  python3 test_basic/current_monitor.py
  ```

  **주의**: `motor.yaml`의 `enable_rtr_requests`가 `false`면 전류(CAN 0x14)를
  아예 요청하지 않아 값이 영영 안 온다. 이 도구가 그 상태를 "전류 미수신"으로
  구분해 표시하지만, 실제로 재려면 그 값을 먼저 `true`로 올려야 한다 —
  그 자체가 GDS68에서 미확인이라 꺼둔 것이므로(주석 참고) 켜고 나서 CAN이
  안정적인지도 같이 봐야 한다.

## 3. BNO086 heading 영점 교정 (검증 도구 구현·5회 실측 완료 — 오프셋 반영 여부 결론 필요)

- **필요성**: IMU는 I2C로 실측 연결 확인됐지만(`imu.yaml`
  `enabled: true`, 주소 0x4B), `heading_offset_deg`가 아직 `0.0`이라
  `imu/heading_deg`가 실제 방위와 어긋날 수 있다. 여름 적재의 `turning`
  단계가 이 값에 의존한다.
- **구현 위치**: `src/base/config/imu.yaml`의 `heading_offset_deg`,
  `src/base/base/imu_aruco_360_test.py`(신규 검증 노드).
- **2026-09-01 검증 방법 변경 및 실측**: "알려진 방위로 정렬 후 육안
  비교" 대신, 아루코 마커를 기준점으로 삼아 자동으로 오차를 재는
  원샷 노드(`imu_aruco_360_test`)를 추가했다 — 로봇을 360도 회전시키며
  IMU 누적 각도를 재고, 회전 전후 같은 마커의 yaw 차이로 실제 오차를
  구한다. 이 방식으로 5회 반복 실측을 완료했다.
- **아직 안 된 것**: `heading_offset_deg`는 이번 실측 이후에도 여전히
  `0.0`으로 남아 있다 — 5회 실측 오차값을 근거로 오프셋을 반영할지,
  아니면 오차가 무시할 수준이라 `0.0` 유지로 결론 낼지 정리가 안 됐다.
  다음에 이 항목을 다룰 때 실측 오차값(도 단위)을 여기 기록하고,
  오프셋 반영 여부를 확정한다.

## 4. `yolo_manager` 재배선 여부 결정

- **필요성**: `mission/yolo_manager`는 완성돼 있지만 `robot.launch.py`에서
  빠져 있고, `vision/yolo/select`를 발행하는 노드가 없어 모델을 올릴 방법이
  없다. 여름 미션은 현재 YOLO 대신 HSV 적색 검출(`cargo_load`)을 쓴다.
- **구현 위치**: `src/mars_launch/launch/robot.launch.py`,
  `src/mission/mission/common/yolo_manager.py`.
- **구현 방향 제안**: 다른 계절(봄/가을/겨울)에서 YOLO가 실제로 필요해지면
  `robot.launch.py`에 노드를 추가하고, `vision/yolo/select`를 발행할
  UI/미션 로직을 먼저 설계한다. 필요 없다면 이 항목은 "보류"로 유지.

## 5. `yolo_models.yaml`의 `model_root`가 다른 워크스페이스를 가리킴 (2026-08-31 저장소 rename으로 우연히 해소됨 — 확인만 남음)

- **필요성(경위)**: `src/mission/config/yolo_models.yaml`의 `model_root`가
  `/home/ubuntu/ros2_ws/models`로 돼 있었다 — 예전엔 이 워크스페이스
  (`ros2_ws_hoyeong`)가 아니라 팀원이 git 없이 쓰던 원본 `ros2_ws`의
  모델 폴더를 가리키는 문제였다(`ros2_ws_huisu`에서 복사해 올 때 경로가
  갱신 안 된 것으로 추정).
- **2026-08-31 갱신**: 이 저장소를 `ros2_ws_hoyeong` → `ros2_ws`로
  이름을 바꾸면서(팀원의 옛 `ros2_ws`는 `ros2_ws_archive_20260831`로
  보관) `model_root` 값 자체는 안 건드렸는데도 **이제 이 저장소 자신의
  `models/`(spring/summer/autumn/winter 하위 폴더 확인됨)를 가리키게
  됐다** — 값을 안 고쳐도 우연히 해소됨. `4번 항목(yolo_manager 재배선
  여부)`이 보류인 동안은 실제 동작에 영향 없다는 점은 그대로다.
- **구현 위치**: `src/mission/config/yolo_models.yaml`의 `model_root`.
- **아직 남은 것**: `yolo_manager`를 실제로 쓰기로 결정하면(4번), 그때
  `models/` 안에 실제 필요한 모델 파일(가중치 등)이 들어있는지 확인한다
  (지금은 경로만 맞을 뿐 폴더 안 내용까진 이 세션에서 확인 안 함).

## 6. `ACTIONS` 상수 중복 — 자동 동기화 없음

- **필요성**: `drive/arbiter.py`의 `ACTIONS`는 `fleet/protocol.py`의
  `ACTIONS`를 수동으로 복사해 둔 것이다(arbiter가 fleet에 빌드 의존하지
  않기 위함, 코드 주석에 명시). 한쪽만 고치면 조용히 어긋난다.
- **구현 위치**: `src/drive/drive/arbiter.py` (`ACTIONS`),
  `src/fleet/fleet/protocol.py` (`ACTIONS`).
- **구현 방향 제안**: 지금 당장 문제는 아니지만, 이 둘 중 하나를 고칠
  일이 생기면 반드시 함께 확인한다 — 자동 테스트(두 상수가 같은지 비교하는
  단위 테스트)를 추가하면 사람이 매번 기억할 필요가 없어진다.

## 7. 독립 롤러(컨베이어) 모터 실제 구동 재검증 (CAN 응답은 복구됨 — 물리 회전만 확인 필요)

- **필요성**: 원래는 "웹 콘솔 수동 컨베이어 조작을 붙였는데 실물이 안
  돈다 → `motor/status`의 `motors.3`(롤러)만 `axis_error`/`axis_state`
  등이 전부 `null`, CAN 하트비트 무응답"이었다. **2026-08-29 로봇 전원을
  껐다 켠 뒤 재확인하니 롤러(ID 3)도 정상 응답한다** —
  `axis_state=1`(IDLE)·`axis_error=0`·`encoder_velocity` 값도 정상으로
  들어온다(10번 항목과 사실상 같은 사건이었을 가능성이 높다 — CAN
  전체가 한꺼번에 죽었다 전원 재인가로 같이 복구됨). `axis_state=1`은
  버그가 아니다 — `motor.py`가 `roller_torque_enabled`를 롤러에 실제
  명령이 들어올 때만 `set_closed_loop()`로 올리도록 짜여 있어서(좌/우는
  초기화 시 바로 closed-loop, 롤러는 명령 대기 중엔 IDLE), 통신만 되면
  이 상태가 정상이다.
- **구현 위치**: `src/base/base/motor.py`(로직은 이미 정상으로 보임,
  수정 아님) — 남은 건 실물 확인뿐.
- **구현 방향 제안**: 웹 콘솔에서 "컨베이어(롤러) 수동 조작" 정/역방향
  버튼을 눌러 (1) `motor/status`의 `motors.3.axis_state`가 `8`
  (CLOSED_LOOP_CONTROL)로 바뀌는지, (2) 실제 컨베이어 롤러가 물리적으로
  도는지 눈으로 확인한다. 둘 다 되면 이 항목은 완료로 옮긴다.

## 8. web_manual.launch.py — 1호기 없는 단독 벤치 시험 + pose 값/3축 오버레이 표시 (구현됨 — 시각 확인만 남음)

- **필요성**: 사용자 요청. `web_manual.launch.py`로 2호기 단독 벤치
  시험을 하는데 ①추종 모드는 1호기의 `/fleet/state`가 있어야만 켜져서
  단독으로는 추종까지 같이 시험할 수 없었고, ②ArUco pose(x/y/z/roll/
  pitch/yaw)는 계산은 되는데 웹 콘솔 어디에도 수치로 안 보이고, 마커
  기준 3축 좌표계를 영상 위에 그려서 눈으로 확인하고 싶다는 요청.
- **구현 위치**:
  - 1호기 없는 추종 시험: `src/fleet/fleet/fake_leader.py`(신규),
    `src/fleet/setup.py`(entry_point), `src/mars_launch/launch/
    web_manual.launch.py`(`start_fake_leader`/`fake_leader_section`
    인자, 기본 켜짐).
  - pose 수치 표시: `src/mars_console/mars_console/templates/unit.html`
    (상태 표에 X/Y/Z(mm)·Roll/Pitch/Yaw 행 추가, `marker/status`의
    기존 필드를 그대로 읽음 — 서버 쪽 변경 없음).
  - 3축 좌표계 오버레이(토글): `src/base/base/aruco_tracker.py`
    (`draw_axes`/`axes_length_mm`, `cv2.drawFrameAxes`), `marker_vision.py`
    (`marker/axes_enable` 구독, `marker/status.axes_enabled` 발행),
    `mars_console/console.py`(`marker_axes_topic`,
    `POST /api/unit/<n>/marker/axes`), `unit.html`(토글 버튼). 기본
    꺼짐 — 실시간성 우선, 필요할 때만 켠다.
- **검증 상태**: 자동화 테스트 통과 —
  `src/mars_console/test/test_manual_solenoid_roller.py`(6개, axes 관련
  2개 신규), `src/mission/test/test_follow_leader_marker_pose.py`(회귀
  없음 확인). **2026-08-29 `web_manual.launch.py` 실물 재시작으로 아래
  전부 확인**:
  - 카메라 스트림 해상도 분리: `/unit2/camera/image/compressed`(검출용)
    1280x720/60061B, `/unit2/vision/image/compressed`(웹 표시)
    640x360/29371B로 실제 라이브 토픽에서 확인 — 검출 해상도는 그대로,
    웹으로 나가는 바이트는 절반 이하로 줄었다.
  - fake_leader: `/fleet/state`에 `source:"fake_leader"` 메시지가 실제로
    올라오고, 콘솔 API의 `fleet_link`/`follow_available`이 1호기 없이
    `true` — `POST /api/unit/2/mode {mode:"follow"}`가 이전처럼 409로
    거부되지 않고 실제로 `mode=follow`/`source=follow`로 전환됨(확인 후
    안전하게 `manual`로 원복).
  - 3축 오버레이 토글: `POST /api/unit/2/marker/axes`로 켜고 끄면
    `marker/status.axes_enabled`가 실제로 따라 바뀜(ROS 토픽에서 직접
    확인).
  - **아직 안 된 것**: 카메라 앞에 실제 ArUco 마커(ID 1)를 놓고 (1) 웹
    브라우저에서 3축 좌표계가 실제로 그려지는지와 X/Y/Z/Roll/Pitch/Yaw
    수치가 화면에 뜨는지, (2) 추종 모드에서 로봇이 마커를 향해 실제로
    움직이는지 — 이 둘은 사람이 마커를 들고 화면을 봐야 확인 가능하다.
    (CAN 통신은 2026-08-29 전원 재인가로 복구 확인됨 — 좌/우 구동 모터
    `axis_state=8`(CLOSED_LOOP_CONTROL) 정상, 자세한 진단 경과는
    [docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md) 참고 —
    이제 추종 주행 확인을 막는 요인이 아니다.)

## 9. 솔레노이드 장시간 통전 과열 (구현됨 — 실물 재검증 필요)

- **필요성**: 2026-08-29, `web_manual.launch.py`를 오래 띄워둔 채 수동
  CAN/모터 시험을 하는 동안 사용자가 솔레노이드가 "엄청 뜨거워진" 것을
  발견했다(다른 세션 `agent/follow-yaw-control-20260829`에서 발견,
  이 브랜치에는 이번에 반영). `solenoid.py`/`solenoid.yaml` 설계상
  "해제"(커플링 안 잠긴 평상시 상태)가 곧 **코일 통전 유지 상태**이고
  (무통전이면 스프링으로 잠기는 페일세이프라 반대 방향은 전류가 안
  든다), 노드는 시작 시·종료 시 모두 이 "해제(통전)" 레벨로 되돌아가게
  짜여 있었다. 즉 커플링을 실제로 안 쓰는 대부분의 시간에도 코일에 계속
  전류가 흐르고, 노드가 죽어도 GPIO가 마지막 통전 레벨 그대로 남을 수
  있어 과열이 구조적으로 누적됐다.
- **2026-08-30 원인 확정 및 수정**: 사용자가 솔레노이드의 실제 물리
  동작을 설명해줬다 — 그냥 밀어 넣으면 스프링 힘으로 잠기는 게
  "잠금"(무통전, 안전한 기본 상태)이고, 솔레노이드가 실제로 걸쇠를
  당겨 내리는 게 "해제"(통전)다. "해제=무통전"이라는 일반적인 통념과
  반대인데, 코드가 정확히 그 통념대로(시작·종료 시 "해제"를 기본값으로)
  짜여 있었던 게 과열의 직접 원인이었다. `src/base/base/solenoid.py`의
  시작 시 GPIO claim과 `shutdown()`을 모두 무통전 레벨(`lock_level`)로
  바꾸고, 헷갈리기 쉬웠던 `off_level`/`on_level` 변수명도
  `release_level`/`lock_level`로 바꿨다(실제 GPIO 값은 그대로,
  `active_low` 계산식 불변). `src/base/config/solenoid.yaml` 주석도
  같이 갱신.
- **구현 위치**: `src/base/base/solenoid.py`, `src/base/config/solenoid.yaml`.
  경위는 [docs/base/solenoid.md](docs/base/solenoid.md) "알려진 이슈"
  3번, [HARDWARE.md](HARDWARE.md) 솔레노이드 행 참고.
- **아직 실물로 확인 못한 것** (다음 로봇 접근 시 최우선 확인):
  1. 이 수정 이후 `robot.launch.py`/`web_manual.launch.py`를 띄운 채
     오래 뒀을 때 솔레노이드가 실제로 안 뜨거워지는지.
  2. `mission/coupling`이 `align` 단계 시작 시 명시적으로
     `solenoid_cmd=false`(해제)를 발행하는 로직이 새 기본값(잠금)과
     맞물려도 자동 결합이 여전히 정상 동작하는지(정렬 전 걸쇠가 실제로
     풀려 있는지, 접촉 후 잠금이 정상적으로 걸리는지).
  3. (참고, 급하지 않음) 이 솔레노이드가 연속 통전(100% 듀티)을
     원래 버틸 수 있는 부품인지는 여전히 사양 미확인 — 다만 평상시
     기본값이 무통전으로 바뀌어 실사용 중 연속 통전 시간 자체가 크게
     줄었으므로 급한 위험은 낮아졌다.
- **2026-09-01 정정 4 — 극성 재반전 + 자동 재잠금 타이머 추가**: 실물에서
  웹 콘솔 UI의 잠금/해제 표시와 실제 걸쇠 동작이 반대로 확인돼
  `active_low`를 다시 `true`→`false`로 뒤집었다(논리 동작은 그대로,
  GPIO 물리 레벨만 반전). 같은 커밋에서 해제 상태 방치를 막는
  `release_timeout_s`(기본 5초) 자동 재잠금 타이머도 추가했다 — 자세한
  내용은 [docs/base/solenoid.md](docs/base/solenoid.md) "알려진 이슈"
  4번 참고. **이 극성 반전으로 위 1·2번("실물로 확인 못한 것")의 검증도
  원점에서 다시 필요하다** — 정정 3 시점 검증이 실제로는 반대 극성으로
  이뤄졌을 수 있다.

## 10. CAN 어댑터·모터 컨트롤러 응답 불안정 재발 — 전원계 물리 점검 필요

- **필요성**: 2026-08-29 하루에만 PCAN-USB가 **19번** 탈부착(`can0
  removed/attached`)됐고, 그 중 두 번(09:59, 11:16)은 `motor` 노드가
  떠 있는 도중 발생해 노드가 죽거나(`exit code 1`) 모터 컨트롤러가
  응답을 멈췄다(`can_connected: false`, `can0` RX 패킷 0인데 버스
  에러 카운터도 0 — 배선 단선이 아니라 컨트롤러 자체가 응답 안 하는
  패턴, 같은 날 앞서 기록된 CAN 장애 이력과 동일 증상). 그때도 지금도
  **소프트웨어로는 못 고치고 로봇 전원을 완전히 껐다 켜야 해결**됐다 —
  같은 시간대에 시스템 자체 재부팅도 여러 번 있었던 것과 묶어 보면
  USB 어댑터·CAN 컨트롤러·라즈베리파이 전원이 같은 전원 계통에서
  같이 흔들리고 있을 가능성이 있다. (다른 세션
  `agent/follow-yaw-control-20260829`에서 발견, 이 브랜치로 반영.)
- **구현 위치**: 소프트웨어가 아니라 **하드웨어 점검** — 배터리/전원
  커넥터, USB-CAN 어댑터 케이블·포트, 모터 컨트롤러(ODrive) 전원 배선.
  참고: [docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md)
  "CAN 버스 장애 이력" 절, `src/base/base/motor.py`(`can_connected`
  판정), udev 규칙 `/etc/udev/rules.d/98-can0-autostart.rules`
  (인터페이스 재열거 자동 복구는 이미 해결됨 — 컨트롤러 무응답은 별개).
  이 udev 규칙 파일은 `/etc`에 있어 이 git 저장소 대상이 아니다 — 로봇을
  새 이미지로 재설치할 때는 다시 만들어 줘야 한다.
- **2026-09-01 소프트웨어 완화(근본 원인 해결 아님)**: `motor.py`가
  탈부착으로 소켓이 죽으면(`[Errno 19] No such device`) 예외를 그대로
  올려 노드가 죽던 것을, 죽은 버스를 놓고 `can_reconnect_interval_s`
  (기본 2초)마다 스스로 재오픈·모터 재초기화하도록 고쳤다 —
  [docs/base/motor.md](docs/base/motor.md) 참고. **이건 "탈부착이 일어나도
  노드가 죽지 않고 돌아온다"는 증상 완화일 뿐, 아래 하드웨어 점검
  (전원계 원인)을 대신하지 않는다** — 탈부착 자체는 여전히 일어나고,
  재연결 사이 구간에는 모터가 응답하지 않는다.
- **구현 방향 제안**: 코드로 재발을 감출 게 아니라 원인을 찾아야 한다 —
  ① 전원 커넥터/배선 접촉 불량 여부 육안 점검, ② 배터리 전압이 로봇
  구동 중(특히 모터 부하가 걸릴 때) 떨어지는지 멀티미터로 실측, ③
  USB-CAN 어댑터를 다른 포트/다른 케이블로 바꿔 재발 빈도가 줄어드는지
  비교. 원인이 좁혀지면 이 항목을 근거로 실제 배선/부품 교체 작업을
  진행한다.
  **주의**: 2026-08-29 12시경 사용자가 모터 컨트롤러 전원을 의도적으로
  끈 채 보드만 급전 중인 상태를 확인했다 — 이 상태에서의
  `can_connected: false`는 당연한 결과이니 이 항목의 근거가 아니다.
  이 항목은 그 이전(09:59·11:16, 모터 전원이 켜져 있던 걸로 추정되는
  시점)의 무응답만을 근거로 한다 — 재확인할 때 그 시점 모터 전원
  상태를 다시 짚어봐야 한다.

## 11. follow_leader — yaw(정면 정렬) 비례 제어항 추가 (구현됨 — 실측 튜닝 필요)

- **필요성**: 사용자가 실제로 추종을 시켜보니 동작은 하는데, 지금 제어가
  거리(z)·좌우 오프셋(x) 두 항만 비례 제어하고 마커의 **yaw(정면 정렬)는
  전혀 반영하지 않는다** — 리더가 제자리에서 돌면 팔로워가 그 회전을
  따라 정렬하지 못한다. 화면 중심-마커 오프셋/z축 거리/마커 yaw 각각에
  독립적인 비례 게인을 주고 통합해 양쪽 모터(L/R) 파워로 나가는 조향값을
  만들어 달라는 요청.
- **구현 위치**: `src/mission/mission/winter/follow_leader.py`
  (`k_yaw`/`yaw_tolerance_deg`, `PoseStamped.orientation`에서 yaw만
  뽑는 역변환 추가), `src/mission/config/follow.yaml`.
- **구현 방향 제안**: 기존 거리 항(`k_linear`, UWB/마커 EMA 폴백)과 좌우
  오프셋 항(`k_angular`, `pose.position.x`)은 이미 실측 튜닝돼 실제로
  잘 도는 상태라 그대로 둔다 — 손대면 검증된 동작이 퇴행할 위험이 크다.
  yaw 항만 새로 추가해 `angular.z = 기존 오프셋항 + k_yaw*yaw_rad`로
  더한다(`drive/arbiter`→`motor.py`가 `linear.x`는 공통 전진력,
  `angular.z`는 좌우 차동으로 이미 나누고 있어, `Twist`에 항을 더하는
  것 자체가 "양쪽 모터 파워에 통합값 반영"과 동등하다 — 별도 L/R 직접
  계산 경로를 새로 안 만들어도 된다). **`k_yaw` 기본값은 0.0(비활성)** —
  부호/크기를 실측하지 않은 새 항이라 이 프로젝트의 다른 미실측 값들
  (`contact_current_threshold_a` 등)과 같은 원칙을 따른다. 실측 순서:
  로봇을 세워두고 마커(리더)를 손으로 천천히 돌려보며 `follow` 모드에서
  `mission/follow/status`의 `yaw_deg`/`angular` 값을 관찰 → 작은 값(예
  0.01)부터 올려 로봇이 마커 정면을 향해 회전하는 방향인지 확인 → 반대
  방향이면 부호를 뒤집는다. `k_yaw` 기본값이 0.0(비활성)이라 이 항 자체는
  `master`에 있어도 실제 조향에는 영향이 없다(자동화 테스트로 회귀 없음
  확인됨) — 그래서 `agent/follow-yaw-control-20260829` 브랜치에서
  `master`로 가져왔다(2026-08-30). **`k_yaw`를 0이 아닌 값으로 켜기
  전에는 반드시 위 실측 순서대로 부호/크기를 확인한다.**
- **2026-08-30 실측으로 축 오인식 발견 및 수정**: 로봇을 실제로 약
  30도 돌려 사진 2장(전/후)을 `cv2.aruco`+`solvePnP`(marker.yaml과
  같은 캘리브레이션 값 사용)로 직접 분석해보니, `aruco_tracker.py`
  기준 `pitch_deg`는 -27.7도 변했는데(≈실제 회전과 거의 일치)
  `yaw_deg`는 -0.26도만 변했다. 원인: `aruco_tracker.py`의
  roll/pitch/yaw는 마커 "자기 자신"의 로컬 축(X=마커 가로변, Y=마커
  세로변, Z=마커 면 법선) 기준일 뿐 로봇 마운트 방향을 모르는데, 이
  로봇은 마커판이 뒤쪽에 **수직으로** 붙어있어서(마커 로컬 Y축 = 세상의
  위쪽) 로봇의 실제 좌우 회전(월드 yaw)이 마커 로컬 **Y축**(코드가
  `pitch_deg`라 부르는 것) 회전으로 나타난다 — 코드가 `yaw_deg`라
  부르는 로컬 Z축 회전은 오히려 로봇의 롤(뱅킹)에 해당해서, 제자리
  회전에는 거의 반응하지 않는 게 정상이었다. `follow_leader.py`의
  `_yaw_from_quaternion`(쿼터니언의 yaw/Z축 성분을 뽑고 있었음)을
  `_heading_from_quaternion`으로 개명하고 쿼터니언의 pitch(Y축) 성분을
  뽑도록 수정 — `k_yaw`가 여전히 0.0이라 이 수정 자체도 실제 조향에는
  영향 없음. 잔차(Δroll -3.4°, Δyaw -0.26°)가 완전히 0이 아닌 건 마커
  마운트가 완벽히 수직은 아닐 수 있다는 뜻 — 급하지 않지만 정밀 보정이
  필요하면 참고. 자동화 테스트(`test_follow_leader_marker_pose.py`)의
  왕복 검증도 pitch_deg를 복원하는지로 같이 수정, 27개 전부 통과.

## 12. ROS_DOMAIN_ID 불일치 의심 — 이 저장소(10) vs 1호기(mars1-rokacup26, 44)

- **필요성**: 이 저장소의 `README.md`, `RUNBOOK.md`,
  `COMMUNICATION_PROTOCOL_UNIT2.md` 전부 `ROS_DOMAIN_ID=10`을 1·2·3호기
  공통값으로 못박아 두고 있고("1·2·3호기가 서로 보이려면 필수"),
  `mars_launch/robot.launch.py`가 실행 시 이 값을 `SetEnvironmentVariable`로
  강제한다. 그런데 자매 저장소 `mars1-rokacup26`(1호기)의 `README.md`/
  `RUNBOOK.md`는 `ROS_DOMAIN_ID=44`를 표준값으로 적고 있다 — 그쪽 커밋
  로그(`26c4aa7`)를 보면 44는 원래 "같은 물리 Orin 위 두 코드 사본(huisu/
  hoyeong 개인 개발본)이 동시에 CAN에 명령을 쏘는 근접사고"를 막으려고
  **격리 목적으로** 정한 값이지, 2·3호기와의 Fleet Protocol 통신을 염두에
  둔 값이 아니었다. 지금 두 저장소 문서 그대로 각자 기동하면, 사용자가
  걱정한 "같은 도메인이라 충돌"이 아니라 **오히려 도메인이 달라 Fleet
  Protocol 자체가 안 열리는 상황**(1호기가 2·3호기 상태를 못 받고, 명령도
  안 감)이 될 가능성이 있다. 실제 대회장에서 마지막으로 어떤 값으로
  기동했는지는 두 저장소 문서만으로는 확정이 안 된다.

  조사해 보니 두 호기가 실제로 주고받는 토픽은 `/fleet/state`,
  `/fleet/cargo/target`, `/fleet/cmd/unit{2,3}`, `/fleet/status/unit{2,3}`
  4개뿐이다(계약은 이 저장소의 `COMMUNICATION_PROTOCOL_UNIT2.md` — 1호기
  저장소에는 사본이 없고 `docs/fleet/protocol.md`가 "규약 문서는 이
  저장소 밖에 있다"고만 적어 둠). **ArUco 마커 쿼터니언(`marker/pose`,
  PoseStamped)은 네트워크로 넘어가지 않는다** — 이 호기가 자기 카메라로
  찍어 `follow_leader` 내부에서만 쓴다(1호기도 자기 카메라로 찍어 결합/
  화물 인식에 로컬로만 쓴다). 두 로봇 사이로 실제 넘어가는 센서값은
  `/fleet/status/unit{unit}`의 `uwb_mm` 필드(이 호기가 측정한 UWB 거리
  숫자) 하나뿐이다 — 즉 UWB 거리는 사용자 추측대로 이 저장소(2호기)가
  발행해서 1호기로 보내는 게 맞고, ArUco 쿼터니언은 애초에 어느 쪽으로도
  네트워크를 안 넘는다. 도메인이 달라도 `cmd_vel`/`marker/pose`처럼
  "같은 이름의 로컬 토픽"이 섞여 오염될 위험은 낮지만, 대신 4개 계약
  토픽이 안 보이면 리더/팔로워 상태 교환 자체가 끊긴다.
- **구현 위치**: 이 저장소는 문서(`README.md`, `RUNBOOK.md`)만 수정
  대상. 실제 도메인 일치 여부는 실물 1호기·2호기에서 각각
  `echo $ROS_DOMAIN_ID`, `ros2 topic list | grep fleet`로 상호 가시성을
  실측해야 확인된다 — 코드 수정이 아니라 확인·조율 작업.
- **구현 방향 제안**:
  1. 실물 1호기·2호기 각각 부팅 시 실제 `ROS_DOMAIN_ID` 확인(`.bashrc`,
     systemd, launch의 `SetEnvironmentVariable`가 셸 값을 덮어쓸 수
     있다는 점 포함).
  2. 이 저장소 규약값(10)으로 맞출지 44로 통일할지 팀이 결정 — 사람이
     정할 사안이라 이번 조사에서 코드/문서를 임의로 바꾸지 않았다.
  3. 확정되면 이 저장소의 `10`과 `mars1-rokacup26`의 `44` 중 stale해진
     쪽을 실제값에 맞게 고친다.
  4. 맞춘 뒤에도 `web_manual.launch.py`/`manual_drive.launch.py`의
     `test_domain`(기본 77) 시험용 오버라이드가 실전 launch에 섞여
     들어가지 않는지 별도 확인한다.
- **2026-08-30 이 저장소(2호기 실기, 호스트명 `unit2`) 쪽 셸 확인 결과**:
  이 세션이 직접 이 로봇(라즈베리파이, hostname `unit2`) 위에서 돌고
  있어 `echo $ROS_DOMAIN_ID`로 실측했다. **`~/.bashrc`의 셸 기본값은
  `11`**로, 이 저장소 문서가 규약값으로 못박은 `10`도 아니고 1호기
  문서의 `44`도 아닌 **제3의 값**이었다 — 지금까지 아무 문서에도 안
  적혀 있던 값이라 그 자체가 stale 발견. 다만 `robot.launch.py`가
  `SetEnvironmentVariable('ROS_DOMAIN_ID', '10')`으로 강제 지정하고
  이 로봇은 systemd 자동시작 없이 매번 사람이 수동으로
  `ros2 launch ...`를 실행하는 구조라(`/etc/systemd/system`에 mars/ros
  관련 서비스 없음, crontab/rc.local도 무관), **실제 로봇 기동 시에는
  셸의 `11`이 아니라 launch가 강제하는 `10`이 적용된다** — 그 자체로는
  버그가 아니다. 다만 launch 없이 맨 셸에서 바로
  `ros2 topic list`/`ros2 run`을 실행하면 도메인 `11`에 격리돼 launch로
  띄운 노드들(`10`)이 하나도 안 보이는 함정이 될 수 있다(정비 중 "토픽이
  하나도 안 보인다"는 착각의 원인이 될 수 있음 — 셸 기본값을 왜 `11`로
  뒀는지 이력은 이 조사로 알 수 없었다).
  - **1호기(`orin`, Tailscale 호스트명 `orin.tail945ffb.ts.net`) 쪽은
    이 세션에서 확인 못함** — Tailscale로 호스트 자체는 보이지만(`tailscale
    status`에 `orin` 응답 있음), `mars`/`ubuntu`/`root`/`orin` 계정
    전부 이 세션이 가진 SSH 키로는 `Permission denied (publickey,
    password)` — 이 저장소(2호기)와 1호기(`orin`)는 별도 계정 자격증명이라
    당연한 결과다(AGENTS.md 1절대로 로그인 재시도는 하지 않음). **1호기
    쪽 실제 `echo $ROS_DOMAIN_ID`와 `robot.launch.py`류의 강제 설정
    여부는 사람이 직접(또는 1호기 접근 권한이 있는 세션이) 확인해야
    한다.**
  - **2026-09-01 이 저장소(2호기) `~/.bashrc` 셸 기본값 `11`→`10` 수정**:
    launch 강제값(10)과 불일치하던 셸 기본값을 저장소 표준값(10)으로
    맞춤 — launch 없이 셸에서 바로 `ros2 topic list` 등을 실행했을 때
    도메인이 갈려 노드가 안 보이던 함정 제거. 1호기(`44`)와의 최종
    통일 여부는 여전히 미결.

## 13. 운용 PC에서 카메라 화면을 웹(MJPEG) 대신 ROS 2 토픽으로 직접 수신 — 지연 개선 검토

- **필요성**: 이 저장소는 `test_console.py`(포트 5000, `/camera.mjpg`)로
  이 호기 정비용 화면을 웹으로 낸다(`docs/drive/test_console.md`) — ROS
  `CompressedImage`(`camera/image/compressed`)를 구독해 Flask가 HTTP
  multipart로 재중계하는 구조로, 1호기 쪽 `mars_console`과 같은 패턴이다.
  이 경로는 "DDS 수신 → Flask 큐잉/멀티파트 래핑 → HTTP → 브라우저 디코드"를
  매 프레임 거치므로, ROS 토픽을 운용 PC가 직접 구독하는 경우보다 지연
  요인(홉)이 하나 더 있다. 사용자가 지연 감소를 목적으로 웹 대신 ROS
  자체로 화면을 받고 싶다고 요청함 — 방향은 타당하나 아래 선행 조건이
  있다. 다만 이 저장소의 `test_console`은 애초에 "정비용" 화면이라
  상시 운용 모니터링 용도(1호기 쪽 `mars_console`)와는 목적이 다르다는
  점은 감안한다.
- **구현 위치**: 이 저장소 코드 변경은 없음(이번 조사는 가능성 확인까지만).
  실제로 켜려면 운용 PC 쪽 설정(이 저장소 밖)이 대부분이고, 이 저장소에
  해당하는 부분은 `src/base/config/camera.yaml`(`jpeg_quality` 등) 조정
  정도뿐이다.
- **구현 방향 제안 (조사 결과)**:
  1. **12번 항목(도메인 일치)이 먼저 해결돼야 한다.**
  2. 이 저장소는 `RUNBOOK.md`/규약 문서 어디에도 Tailscale 언급이 없고,
     시험 화면 접속 주소가 LAN 고정 IP(`http://10.10.2.42:5000`)로
     적혀 있다 — 즉 운용 PC가 이 호기와 **같은 LAN**에 있는 구성으로
     보인다. 이 경우 1호기(Tailscale VPN 경유)와 달리 DDS 기본
     멀티캐스트 discovery가 별도 설정 없이도 될 가능성이 있다(다만
     실측 확인 필요 — 라우터/AP의 멀티캐스트 차단 여부에 따라 다르다).
     원격(WAN)에서 접속해야 한다면 1호기 쪽 14번 항목과 같은 이유로
     Tailscale 등 VPN에서는 멀티캐스트가 기본적으로 안 되므로 별도
     유니캐스트 discovery 설정(CycloneDDS `Peers`, Discovery Server,
     또는 `rmw_zenoh_cpp`)이 필요하다.
  3. **운용 PC 준비물**: 로봇과 같은 ROS 2 배포판(Jazzy) 설치. 이 저장소를
     colcon build할 필요는 없고, `rqt_image_view`나 Foxglove Studio
     (`foxglove_bridge` 노드를 이 호기 쪽에 추가로 띄워야 함, 현재
     미설치) 정도로 `camera/image/compressed`(호기 네임스페이스 기준
     상대 토픽, 실제로는 `/unit2/camera/image/compressed`)를 바로 볼
     수 있다.
  4. **결론**: 가능하고, 이론적으로는 웹 MJPEG보다 지연이 낮다. 다만
     "설정 몇 줄"이 아니라 도메인 통일(12번) + (원격 접속이라면) DDS
     discovery 방식 결정을 먼저 정해야 실제로 켤 수 있다.
- **2026-08-30 "제3의 노트북"(운용 PC가 아니라 별도 노트북, Tailscale 경유)
  + 현재 이 로봇의 RMW 구성으로 실측 가능성 확인**:
  - 이 로봇(2호기 실기)에 설치된 RMW는 **`rmw_fastrtps_cpp`(Fast DDS)
    하나뿐** — CycloneDDS·`rmw_zenoh_cpp` 둘 다 미설치, `RMW_IMPLEMENTATION`
    환경변수도 비어 있어(시스템 기본값 사용) 사실상 Fast DDS로 고정된
    상태. 기존 Fast DDS discovery(XML profile의 `initialPeersList`)나
    Discovery Server 설정 파일도 이 저장소·시스템 어디에도 없다 — 지금은
    **기본 SPDP 멀티캐스트 discovery 그대로**다.
  - `tailscale0` 인터페이스 자체는 `ip link`상 `MULTICAST` 플래그가 붙어
    있지만, 이건 인터페이스 능력 표시일 뿐 실제 전달 여부와 다르다 —
    Tailscale(WireGuard 기반 오버레이)은 각 피어 쌍이 point-to-point
    유니캐스트 터널이라 **피어 간 멀티캐스트/브로드캐스트를 중계하지
    않는 게 알려진 제약**이다. 즉 **지금 Tailscale 연결 상태 그대로는
    기본 discovery가 노트북↔로봇 간에 서로를 못 찾을 가능성이 높다**
    (이 세션에선 Tailscale 반대편에 실제 ROS 2 노드가 있는 노트북이
    없어 pub/sub까지 직접 실측은 못 했다 — 위는 설치된 RMW·discovery
    설정 부재와 Tailscale의 문서화된 멀티캐스트 제약에 근거한 결론).
  - **가능하게 하려면 필요한 추가 작업** (이 저장소 코드 변경은 없고
    로봇·노트북 양쪽 ROS 2 설정):
    1. 도메인 통일(12번, 진행 중) — 전제 조건.
    2. Fast DDS를 유니캐스트 discovery로 바꾸는 XML profile을 만들어
       로봇·노트북 양쪽에 `FASTRTPS_DEFAULT_PROFILES_FILE`로 지정
       (`initialPeersList`에 서로의 Tailscale IP 명시) — 또는 별도
       Fast DDS Discovery Server(`fastdds discovery` 데몬)를 하나 띄우고
       양쪽이 그걸 가리키게 하는 방법도 가능. CycloneDDS `Peers`나
       `rmw_zenoh_cpp`로 갈아타는 것도 대안이지만 지금 이 로봇엔 둘 다
       미설치라 그러려면 패키지 설치부터 필요.
    3. 노트북 쪽에 ROS 2 Jazzy 설치(3번 하위 항목과 동일).
  - **결론**: 카메라 화면을 ROS로 노트북에서 직접 보는 것 자체는 코드
    변경 없이 가능한 방향이 맞지만, **지금 이대로의 Tailscale 연결만으로는
    안 된다** — 위 discovery 설정을 먼저 해야 한다.

## 14. RC(무선 조종기) 모드 — 1호기 경유 RC 채널 값으로 2호기 조종 (구현됨 — 실물 검증 필요)

- **필요성**: 사용자 요청. RC 수신기는 물리적으로 **1호기에만** 연결돼
  있다. 2호기에도 RC로 직접 조종하고 싶은데 수신기를 따로 달지 않고,
  1호기가 받은 채널 값을 네트워크 토픽으로 2호기에 넘겨 그걸로 조종하게
  해 달라는 요청.
- **2026-08-30 1호기 쪽 실제 설계 확인 및 채널 재배치 합의**: 1호기가
  이미 `/receiver/channels`(`UInt16MultiArray`, 14채널)를 직접 방송하고
  있어(호기별로 안 나뉨), 2호기는 그걸 그대로 구독해 CH7(호기 선택
  스위치)로 스스로 자기 몫을 거른다 — 애초에 짐작했던 `/fleet/rc/unit{N}`
  중계 토픽은 필요 없다. 최종 채널 배치·정책은 아래(구현 위치의
  [rc_bridge.md](docs/fleet/rc_bridge.md) 표가 최신 출처):
  - CH1/CH2: 주행(angular/linear), CH3: **솔레노이드**(1호기 카메라 틸트
    채널 재사용, 중앙값 임계 — 스틱 내림=잠금/올림=해제),
    CH5(VRA): 속도 스케일(0~1, 1호기 `knob_scale()`과 동일), CH6(VRB):
    폐기, CH7: 호기 선택(≥1650=2호기), CH8: 정지/재생(≥1650=재생),
    CH9: 컨베이어 3단 스위치, CH10: 자동/수동(≥1500=auto, 2호기 선택
    중일 때만 `drive/mode_cmd`를 원격으로 바꿈).
  - **CH7 이탈 시 정책(사용자 확정)**: 마지막 값을 **유지**한다. 0으로
    끊기는 건 오직 `/receiver/channels` 토픽 자체가 끊겼을 때뿐 —
    이때는 `cmd_vel/rc`(arbiter `source_timeout_s`)와
    `roller_manual_cmd`(`cargo_load` 자체 timeout)가 각자 알아서
    0이 되고, 솔레노이드만 자체 timeout이 없는 래치라 이 노드가
    `signal_timeout_s`(기본 1초) 뒤 직접 안전 잠금으로 되돌린다.
  - **웹/RC 조종 방식 토글(사용자 확정)**: 웹 콘솔에 새 토글
    (`rc/enabled`)을 추가해 "지금 RC 채널을 반영할지"를 사람이 직접
    켜고 끈다. 이 토글을 RC→웹으로 끄는 순간은 CH7 이탈과 달리 **즉시**
    주행/컨베이어를 0으로 만든다(사람이 명시적으로 통제권을 가져간
    것이므로 "유지"하지 않는다) — 솔레노이드는 이 경우에도 안 건드림.
  - CH8('정지')은 `safety/stop`처럼 다른 노드가 이미 쓰는 공유 안전
    토픽에 안 쓴다(다중 발행자 경쟁 상태 위험) — rc_bridge 자기 안에서만
    `cmd_vel/rc`/`roller_manual_cmd`를 0으로 묶는 국소 게이트로 뒀다.
  - VRA는 1호기 `manual.py`의 `knob_scale()` 공식(1000→0.0, 2000→1.0
    선형, 데드존 없음)을 그대로 가져왔다.
- **구현 위치**:
  - `src/fleet/fleet/protocol.py`(`TOPIC_RECEIVER_CHANNELS`,
    `/receiver/channels` — 1호기 실제 이름 확인 후 확정),
    `src/fleet/fleet/rc_bridge.py`(CH1~CH10 전체 처리, "유지"/워치독/
    웹 토글 로직), `src/fleet/config/rc_bridge.yaml`(채널 인덱스·임계값
    전부).
  - `src/drive/drive/arbiter.py` — `MODES`/`SOURCES`에 `'rc'`,
    `drive/status.rc_link`(지난 턴에 구현, 이번 턴엔 무변경).
  - `src/mars_launch/launch/robot.launch.py`(rc_bridge 상시 실행),
    `web_manual.launch.py`(`start_fleet` 게이트).
  - `src/mars_console/mars_console/console.py`(`rc_enabled_topic`,
    `set_rc_enabled()`, `POST /api/unit/<n>/rc/enabled`, `snapshot()`의
    `rc_enabled`/`rc_robot_selected`/`rc_run_active` — 전부 rc_bridge의
    `rc/status`를 그대로 반영, 콘솔 자체 기억 아님), `templates/unit.html`
    ("조종 방식" 토글 패널, RC 켜져 있을 때 "모드" 버튼 잠금).
  - 문서: [docs/fleet/rc_bridge.md](docs/fleet/rc_bridge.md)(전면 갱신),
    [docs/fleet/protocol.md](docs/fleet/protocol.md),
    [docs/ui/console.md](docs/ui/console.md),
    [docs/base/solenoid.md](docs/base/solenoid.md)(발행자 3곳 공유 명시).
  - 테스트: `src/fleet/test/test_rc_bridge.py`(20개 — 정규화/knob_scale/
    컨베이어/솔레노이드 방향/CH7·CH8·CH10 게이팅/"유지"/토글 즉시 0/
    워치독), `src/drive/test/test_arbiter_rc_mode.py`(6개),
    `src/mars_console/test/test_rc_mode.py`(7개, 토글 반영 패턴 포함).
    총 58개 통과, `RUNBOOK.md` pytest 명령에 반영 완료.
- **아직 안 된 것**:
  1. ~~`/receiver/channels`가 실제로 이 이름·타입으로 오는지~~ —
     **2026-09-01 실물로 확인 완료.** `ROS_DOMAIN_ID=10`에서
     `ros2 topic info /receiver/channels --verbose`로 직접 구독해보니
     발행자 노드 `manual`, 타입 `std_msgs/UInt16MultiArray`, **~20Hz**로
     정상 발행 중이었다(도메인 불일치 아님 — 12번 항목 우려 해소).
     받은 실제 샘플: `[1500,1499,1524,1500,1294,1000,1000,2000,1500,
     1000,1500,1500,1500,1890]`(CH1~14 순서).
  2. CH1/CH2/CH3/CH7/CH8/CH9/CH10 각 raw 값과 임계값이 표대로 오는지 —
     위 샘플 기준 CH7(호기 선택)=1000으로 임계값(1650) 미달 →
     "2호기 아님" 판정 구간이었다. **이 스냅샷이 찍힌 순간 조종기의
     호기 선택 스위치가 2호기 쪽으로 안 넘어가 있었다는 뜻** — 아래
     "2026-09-01 1차 실물 시도"의 무반응 원인이 웹 토글/motor 크래시가
     아니라 단순히 이 스위치 위치였을 가능성이 새로 생겼다. 곧이어
     스위치를 2호기 쪽으로 넘긴 뒤 재확인하니 CH7=2000으로 정상 상승
     (아래 "구독 자체 검증" 참고).
     CH1/2/3/8/9/10도 표대로 해석되는 것으로 보이나(CH8=2000 재생,
     CH10=1000 rc 요청, CH5 VRA=1294→스케일 0.29 등), "2호기 선택" 상태의
     정지/모드/컨베이어 반응까지 이어지는 실측(노드 켜고 실제 주행/
     컨베이어 동작 확인)은 아직 못 했다.
  3. ~~구독 자체가 되는지(토픽명·타입·네임스페이스)~~ — **2026-09-01
     실물로 확인 완료.** `TOPIC_RECEIVER_CHANNELS`가 절대경로(`/receiver/
     channels`)라 rc_bridge가 `/unit2` 네임스페이스로 떠도 영향 없음을
     코드로 먼저 확인하고, 이 저장소 빌드의 `rc_bridge` 노드를 단독으로
     3초간 실제로 띄워 `rc/status`를 봤더니 `connected:true, age_s:0.04`
     — 지금 발행 중인 1호기 채널을 정상적으로 받고 있었다(액추에이터
     노드는 안 띄웠으니 하드웨어 동작 없음). 확인 후 바로 종료.
  4. CH1/CH2/CH3 방향(`steering_reversed`/`throttle_reversed`/
     `solenoid_reversed`)이 맞는지 — 반대면 yaml 플래그만 뒤집는다.
  5. 3호기 CH7 선택 밴드 미정의(1/2호기 2단 스위치 기준이라 3호기까지
     구분하려면 재설계 필요) — 3호기 배치 시점에 다시 다룬다.
- **2026-09-01 1차 실물 시도 — "2호기가 RC에 전혀 반응 안 함", 원인 미확정
  (로그 근거로 유력 후보 2개 좁힘)**: `~/.ros/log`의 오늘자 `unit2` 세션
  로그(`KIRO_rasp_ws_dongho` 빌드, 13:42~13:51 사이 4회 실행)를 사후
  확인한 결과:
  1. **마지막 시도(13:50:29 세션, pid 15184)에는 rc_bridge 로그에 "웹
     토글 ON" 줄이 없다** — `enabled` 기본값이 `False`라(`rc_bridge.py`
     `self.enabled = False`), 콘솔의 "조종 방식" 토글을 RC로 켜지 않으면
     `/receiver/channels`가 와도 `_on_channels`가 조용히 버린다(코드 250번
     줄 `if not self.enabled: return`). 이 세션 바로 앞 13:42:14 세션
     로그에는 실제로 "웹 토글 ON — 조종기 입력 반영 시작"이 찍혀 있어
     — 즉 그 시점엔 켰다가, 이후 노드를 재시작하면서(`enabled`는 노드
     프로세스 메모리 상태라 재시작 시 매번 다시 꺼짐) 마지막 시도 때는
     안 켰을 가능성이 있다.
  2. **같은 마지막 세션에서 motor 노드가 기동 8초 만에 죽었다** —
     `can0`에서 `CAN transmit unavailable; retrying with backoff: Failed
     to transmit: No such device or address [Error Code 6]`. rc_bridge가
     `cmd_vel/rc`를 정상 발행했더라도 motor 노드가 죽어 있으면 실제
     구동은 애초에 불가능하다. 이 순간 이후 `can0` 인터페이스 자체가
     내려간 것으로 보이나, 재확인 시점(`ip -details link show can0`)에는
     `state UP`/`ERROR-ACTIVE`로 정상이라 — USB-CAN 동글(`pcan_usb`)의
     순간적 접촉 불량/드라이버 재열거 추정, 재현 여부 미확인.
  3. 두 후보 다 "RC 채널 자체가 안 왔다"는 아니다 — rc_bridge 쪽에
     "채널 수 부족"/"채널 수가 다르다" 경고 로그가 이 4개 세션 어디에도
     없어(그런 경고가 있었으면 남았을 것), `/receiver/channels`가 오긴
     왔다면 채널 개수는 기대대로였을 가능성이 있다. 다만 로그만으론
     `rc/status`(연결 여부/`robot_selected`)가 그 순간 뭐였는지 확정할
     수 없다 — 노드가 껐다 켰다 하는 동안 실시간으로 안 남겼기 때문.
  - **다음 실물 시도 때 확인할 것**: (a) 콘솔에서 "조종 방식"을 RC로
    켠 뒤 `rc/status`를 직접 echo해 `enabled`/`connected`/
    `robot_selected`가 기대대로 뜨는지, (b) motor 노드가 살아있는 채로
    유지되는지(`can0` 드롭 재현 여부 — 아래 20번 참고).

## 15. cargo_load.py — `/fleet/cargo/target` 신선도 검사 누락

- **필요성**: 2026-08-30 `cargo_load.py`/`coupling.py` 코드 점검 중 발견.
  이 파일의 다른 입력값(`uwb_mm`, `heading`, `current_a`)은 전부
  타임스탬프를 찍어 `sensor_timeout_s` 안에서만 "ready"로 쓰는데
  (`coupling.py`의 `marker_control_timeout_s`도 같은 패턴), `cargo_target`
  만 이 패턴이 빠져 있다. `_on_target`은 `detected: true`일 때만
  `self.cargo_target`을 갱신하고 `false`가 와도 지우지 않으며, `approach`
  단계는 `self.cargo_target is not None`만 볼 뿐 "지금도 유효한 표적인가"는
  확인하지 않는다. 1호기가 한 번 표적을 잡아 보낸 뒤 놓쳐서
  (`detected: false`)를 알려와도 2호기는 그 값을 계속 기억하고 있다가
  오래된(어쩌면 오탐이었던) 거리를 향해 계속 접근할 수 있다. 관련
  자동화 테스트도 없다.
- **구현 위치**: `src/mission/mission/summer/cargo_load.py`의
  `_on_target()`(46, 94~99행 부근), `_tick()`의 `approach` 단계(157행
  부근), `src/mission/config/cargo.yaml`(새 timeout 파라미터 추가 시).
- **구현 방향 제안**:
  1. `_on_target`에 `self.cargo_target_time = time.monotonic()` 타임스탬프
     추가(다른 `_on_*` 콜백들과 동일 패턴).
  2. `cargo_target_timeout_s` 파라미터(예: `sensor_timeout_s`와 같은 기본값
     0.5) 추가, `_tick()`에 `cargo_target_ready` 계산(`uwb_ready`/
     `heading_ready`와 같은 형태)해서 `approach` 단계 진입 조건에 포함.
  3. `detected: false` 메시지를 받으면 명시적으로 `cargo_target`을
     무효화할지, 아니면 timeout에만 맡길지는 구현 시 결정 — 전자가 더
     즉각적이지만 1호기 쪽이 실제로 `detected: false`를 주기적으로 보내는지
     (아니면 그냥 메시지 자체를 안 보내는지) 확인 필요.
  4. `src/mission/test/`에 `_on_target`/`approach` 신선도 검사 테스트 추가
     (기존 `test_cargo_load_manual_roller.py`와 같은 스타일).
  5. 미션 자동 주행 로직 변경이라 구현 시 새 브랜치를 판다(AGENTS.md 4절 —
     하드웨어 구동 코드 변경).

## 16. arbiter.py `_source()` — follow 모드에서 화물 명령이 와도 cargo 소스로 안 바뀌는 버그 (구현됨 — 실물 검증 필요)

- **필요성**: `~/ros2_ws`(팀원이 git 없이 작업 중인 별도 워크스페이스)와의
  비교 검토(2026-08-30~31) 중 발견. 지금 `_source()`는
  `if self.mode == 'follow': return 'follow'`로 고정돼 있다 — `follow`
  모드(1호기 추종 중, 여름 이동 구간에 자연스러운 상태) 동안 1호기가
  `/fleet/cmd/unitN`으로 화물 명령(`approach`/`load`/`hold`/`release`)을
  보내면, `mission/action`은 정상 갱신되어 `cargo_load.py`가 내부적으로
  단계를 진행하지만 arbiter는 계속 `cmd_vel/follow`만 선택해 **실제
  모터는 안 움직인다.** `~/ros2_ws` 쪽에 이미 이 버그를 고친 흔적(주석
  포함)이 있어 참고했다.
- **구현 위치**: `src/drive/drive/arbiter.py`의 `_source()`.
- **구현 방향 제안**:
  ```python
  if self.mode == 'follow':
      action = self.action()
      return 'cargo' if action in CARGO_ACTIONS else 'follow'
  ```
  `CARGO_ACTIONS`는 파일 상단에 이미 정의돼 있다. 하드웨어 구동 코드라
  새 브랜치(`agent/arbiter-cargo-fix-20260831`)에서 작업하고, 자동화
  테스트(최소 시뮬레이션 검증)로 확인한 뒤 병합한다 — 실물 검증은
  여름 적재 통합 시험 때 별도 확인.
- **2026-08-31 구현·병합 완료**: `_source()` 수정, `src/drive/test/
  test_arbiter_follow_cargo_source.py` 신규(7개 통과, 회귀 없음 확인 —
  전체 65개 통과), `docs/drive/arbiter.md` 갱신. `agent/arbiter-cargo-fix-
  20260831` 브랜치에서 master로 merge, 브랜치 삭제. **아직 안 된 것**:
  여름 적재 통합 시험 때 `follow` 모드로 이동 중 실제 화물 명령을 받아
  모터가 움직이는지 실물 확인.

## 17. marker.yaml `target_id: -1` → `1` (방어적 고정) (구현됨)

- **필요성**(2026-08-30 시점 기록 — 아래 `~/ros2_ws`는 **팀원의 옛
  git 미관리 작업 디렉터리**를 가리킨다. 2026-08-31 이 저장소 자체가
  `ros2_ws_hoyeong`에서 `ros2_ws`로 이름이 바뀌면서 그 팀원 워크스페이스는
  `ros2_ws_archive_20260831`로 옮겨졌다 — 아래 문단은 당시 그대로 둔다):
  `~/ros2_ws`의 `marker.yaml`은 `target_id: 1`로 명시 고정돼
  있고 "1호기 뒤 ArUco (2026-08-30 확정)" 주석이 있다. 이 저장소는 아직
  `-1`(화면에 보이는 가장 큰 허용 ID를 자동 선택)이다. 자매 저장소
  (`mars1-rokacup26`/`ros2_ws_hoyeong`) TODO 12번은 "물리 마커가 1개·ID
  1뿐이라 -1도 안전하다"고 결론 냈지만, 나중에 2·3호기용 마커가 실제로
  추가되면 `-1`은 화면에 잠깐 더 크게 잡히는 엉뚱한 마커를 따라갈 위험이
  생긴다 — 미리 `1`로 방어적으로 고정해 두기로 결정(2026-08-31).
- **구현 위치**: `src/base/config/marker.yaml`의 `target_id`.
- **구현 방향 제안**: 값만 `1`로 바꾸고, 이 항목과 자매 저장소 TODO 12번을
  서로 참조하도록 주석/문서에 "왜 -1이 아니라 1인지" 이유를 남긴다.
  하드웨어를 직접 구동하는 코드는 아니지만(추종 대상 선택 설정) 안전
  관련 설정이라 16번과 같은 브랜치에서 같이 처리한다.
- **2026-08-31 구현·병합 완료**: 값 변경 + `docs/base/marker_vision.md`
  갱신, 16번과 같은 브랜치에서 master로 merge. 물리 마커가 지금 1개뿐이라
  동작상 차이는 없다(방어적 변경) — 별도 실물 재검증 없이 완료로 본다.
  다음에 로봇을 켤 때 마커 인식이 평소대로 되는지 정도만 눈으로 한번
  확인하면 충분하다.

## 18. Fleet Protocol v1.2 포팅 — `turn`/`couple` 액션, 명령 timeout 2.5초 (구현됨·master 병합 완료 — 실물 통합 시험 필요)

- **필요성**: `~/ros2_ws`의 `fleet/protocol.py`가 이미 "v1.2 — 1호기
  fleet/protocol.py와 짝을 맞춘다"는 docstring과 함께 `ACTIONS`에
  `turn`(angle_deg 동반)·`couple`을 추가하고 `CMD_TIMEOUT_S`를 1.0→2.5로
  올려 뒀다(1호기 leader의 1Hz 재발행 주기 대비 1초는 빠듯하다는 근거).
  **2026-08-31 사용자 확인**: 1호기 쪽(`mars1-rokacup26`) 이 v1.2 구현이
  실제로 존재하되, `main`/`dev/hoyeong`이 아니라 그 서버의 넷째 로컬
  워크스페이스 `/home/mars/ros2_ws`(브랜치 `comp-upgrade-20260830`)에만
  있고 **아직 origin에 push 안 된 상태**(그쪽 저장소 TODO 17번으로
  추적 중, 이 세션은 orin에 접근 권한이 없어 직접 확인은 못 했고 사용자
  보고에 따름). 즉 이 포팅 자체는 지금 착수해도 되지만, **1·2호기 실물
  상호 검증은 1호기 쪽이 push·병합된 뒤에만 의미가 있다** — 코드는
  먼저 짜 두되, 통합 시험 일정은 그쪽 완료를 확인하고 잡는다.
- **구현 위치**:
  - `src/fleet/fleet/protocol.py` — `ACTIONS` 확장, `turn`의 `angle_deg`
    검증(-360~360), `CMD_TIMEOUT_S` 2.5로.
  - `src/drive/drive/arbiter.py` — `ACTIONS`/`CARGO_ACTIONS`에 `turn`/
    `couple` 추가(fleet에 빌드 의존 안 하려고 수동 복사해 둔 사본,
    protocol.py와 반드시 함께 고칠 것 — TODO.md 6번 참고),
    `_clean()`에 `turn`의 `angle_deg` 검증 추가.
  - `src/mission/mission/summer/cargo_load.py` — 지금 이미
    `turn_speed_rps`/`turn_target_deg`/`turn_tolerance_deg`로
    **자기완결형 180도 회전**을 갖고 있다. 완전히 새로 짜는 게 아니라
    기존 로직을 1호기가 보내는 `turn`(임의 `angle_deg`)/`couple` 명령이
    트리거하도록 다시 배선하는 작업이다 — 기존 phase machine(search/
    turning 등)과 어떻게 합칠지 설계를 짧게 정리하고 시작한다.
  - `src/mission/mission/common/coupling.py` — `couple` 명령 수신 시
    180도 회전 → `coupling/cmd`(`start`)로 자동 연결.
  - 3호기 관련 서술 불일치(v1.2 docstring은 "여름 적재는 1·2호기
    둘이서"라는데 `robot.launch.py`/`console.py`엔 3호기 지원이
    그대로 남아 있음 — `~/ros2_ws`도 이 불일치가 그대로 있었다) — 포팅
    하면서 정리하거나, 애매하면 별도 TODO로 남긴다.
  - 콘솔 UI 변경 불필요 — `turn`/`couple`은 1호기가 자동으로 보내는
    명령 전용이라 웹 콘솔에 버튼을 만들지 않는다(`~/ros2_ws`도 동일).
- **작업 순서**: 16→17→18 권장(16은 독립적이고 위험 낮음, 17은 사소함,
  18은 설계 논의가 필요하고 1호기 쪽 push를 기다려야 완전히 검증됨).
  진행 상황·완료 여부는 이 저장소의 TODO 생애주기(구현됨→테스트
  필요→검증 후 삭제)를 그대로 따른다.
- **2026-08-31 설계 정리(착수 전)**:
  1. **`turn` 액션**: `_tick()`의 기존 `turning` phase 로직(IMU heading
     으로 `angle_delta` 비교하며 회전)을 그대로 재사용한다. 목표각을
     yaml 고정값(`turn_target_deg`) 대신 명령의 `angle_deg`로 받도록
     인스턴스 속성 `self.turn_target_deg`을 두고, 진입 경로별로 분기한다
     — 기존 자동 경로(`search`에서 적색 검출 → `turning`)는 지금처럼
     yaml 기본값을 쓰고, 새 명령 경로(`turn` 액션)는 전달받은 각도를
     쓴다. 완료 후 다음 phase도 경로별로 다르다 — 자동 경로는 기존대로
     `backing`으로, 명령 경로는 `idle`로 돌아가 1호기의 다음 명령을
     기다린다(플래그 `self.turn_via_command`로 두 경로를 구분).
  2. **`couple` 액션**: 별도 회전을 다시 하지 않는다 — `turn` 액션이
     이미 독립적으로 방향을 잡을 수 있으므로, `couple`은 `coupling/cmd`
     에 `{"action":"start"}`를 발행하는 것만으로 충분하다.
     `coupling.py`는 이미 이 토픽/액션을 받아 자체 정렬·접촉·잠금
     상태기계를 돌리므로 **coupling.py 쪽 코드 변경은 불필요**하다.
     `~/ros2_ws`는 `couple` 안에 180도 회전까지 묶어 뒀지만, 이 저장소는
     회전(`turn`)과 결합 시작(`couple`)을 분리된 조합 가능한 명령으로
     유지하기로 한다 — 1호기가 `turn`(180)을 보내 완료를 기다린 뒤
     `couple`을 보내는 2단계 시퀀스가 된다. 좁은 설계 차이지만, 이미
     독립적으로 만들 `turn` 액션을 `couple`에 다시 흡수시키지 않는 편이
     phase machine을 더 단순하게 유지한다 — **`~/ros2_ws`와 다른 이
     저장소만의 판단**이니 1호기 쪽과 실제 통합 시험 전에 이 차이를
     인지하고 있을 것.
  3. **1호기로의 상태 보고는 이미 준비돼 있다**: `arbiter.py`의
     `_phase()`가 `source=='coupling'`일 때 coupling의 내부 phase를
     이미 `approach`/`contact`/`locked`/`error` 어휘로 매핑해 두고
     있다(과거에 이 통합을 예상하고 만들어 둔 것으로 보인다) —
     `follower.py`를 거쳐 `/fleet/status/unit{N}.phase`로 그대로
     나가므로 추가 배선이 필요 없다.
  4. `protocol.py`/`arbiter.py`의 `ACTIONS`/`CARGO_ACTIONS`에 `turn`/
     `couple`을 추가해야 16번 수정 덕분에 `follow` 모드에서도 이
     액션들이 `cargo` 소스로 라우팅된다 — 안 하면 명령은 오는데
     cmd_vel은 여전히 `follow`만 쓰이는, 16번과 똑같은 유형의 문제가
     `turn`/`couple`에서 재발한다.
- **2026-08-31 구현 완료·master 병합 완료**: `protocol.py`/
  `arbiter.py`/`cargo_load.py` 전부 위 설계대로 반영, `coupling.py`는
  설계대로 변경 없음. `follower.yaml`의 `command_timeout_s`도 2.5로
  맞췄다. 테스트 22개 추가(protocol 6, arbiter 7, cargo_load 7 — 자기
  완결형 search→turning→backing 경로가 그대로 유지되는지 회귀 확인
  포함). 문서 갱신: `docs/fleet/protocol.md`, `docs/fleet/follower.md`,
  `docs/drive/arbiter.md`, `docs/mission/cargo_load.md`,
  `docs/mission/coupling.md`. 브랜치
  `agent/protocol-v1.2-turn-couple-20260831`에서 시작해 **1호기 쪽
  (`mars1-rokacup26`) v1.2 구현이 origin에 push된 것을 사용자가 확인해줘서**
  병합 보류 조건이 해소돼 master로 merge(브랜치는 유지, 필요시 삭제
  가능). merge 후 `colcon build` 재빌드 + 전체 자동화 테스트 87개 재확인,
  회귀 없음. **아직 안 된 것**: 이 코드는 자동화 테스트만 통과한 상태이지
  실제 1호기와 `turn`/`couple` 명령을 주고받는 **실물 통합 시험**은 아직
  — 여름 적재 통합 시험 때 1호기가 실제로 `turn`(각도)·`couple` 명령을
  보내 2호기가 정상 반응하는지 확인해야 한다.

## 19. `coupled_drive` — 결합 이후 1호기 속도 복제 협조 주행 (구현됨 — 실물 검증 필요)

**(2026-09-04 대체)**: 이 항목이 묻던 "3호기 삭제가 확정 결정인지"는
그 뒤 확정·실행됐다 — 3호기 지원은 `dev/dongho/drop_unit3` 병합으로
저장소에서 실제로 제거됐다(`robot.launch.py`의 `unit` 인자,
`console.py`의 `UNITS[3]` 정리 완료). 아래는 이 번호를 이어받은 새
항목이다.

- **필요성**: 결합(`locked`)이 끝난 뒤 2호기가 1호기와 함께 이동해야
  하는 구간(여름은 적재 제외 구간 전체, 겨울은 제설 구간)이 있다.
  기존 `follow_leader`는 마커 거리를 `target_distance_mm`로 맞추는
  거리 제어인데, 결합 중에는 결합 로드가 거리를 물리적으로 이미
  고정하고 있어 거리 제어기를 그대로 쓰면 로드와 싸운다 — 거리 제어가
  아니라 **속도 복제**가 필요했다.
- **구현 위치**: `src/mission/mission/common/coupled_drive.py`(신규),
  `src/mission/config/coupled.yaml`(신규), `src/drive/drive/arbiter.py`
  (`coupled` 모드 추가), `src/mars_launch/launch/robot.launch.py`(노드
  등록). 문서: [docs/mission/coupled_drive.md](docs/mission/coupled_drive.md),
  [docs/drive/arbiter.md](docs/drive/arbiter.md),
  [COMMUNICATION_PROTOCOL_UNIT2.md](COMMUNICATION_PROTOCOL_UNIT2.md)
  "결합 이동" 절.
- **2026-09-01 구현 완료**: 1호기 속도를 그대로 복제해
  `cmd_vel/coupled`로 낸다. 끊기면 안전하게 0을 낸다(마지막 값 유지 안 함).
  `require_locked`(기본 true)로 결합 중이 아니면 안 굴리고, `linear_sign`
  으로 부호를 보정한다. `coupled_command()`(순수 함수)만 단위 테스트로
  검증했다(`src/mission/test/test_coupled_drive.py`).
- **2026-09-04 채널 교체 — 원래 배선이 아예 안 붙어 있었다**: v1.0은
  `/fleet/state` JSON의 `leader_linear`를 읽기로 했는데, 1호기
  `fleet/protocol.py:make_state`는 그 필드를 실은 적이 없다. 두 호기가 서로
  다른 채널로 말하고 있었고 2호기는 계속 `leader_velocity_missing`으로 0만
  냈다. `/fleet/state`는 5 Hz 상태 알림이라 실려 있었더라도 결합 주행에는
  10배 느리다. 1호기가 이미 내고 있는 `/fleet/cmd_vel/unit2`(Twist, 50 Hz)를
  구독하도록 바꾸고 QoS를 1호기와 똑같이 맞췄다. 통신 규약을 v1.1로 올렸다
  (필드 삭제이므로).
- **2026-09-04 `follow_angular` 켬**: 결합부가 강체로 확인됐다. 강체의
  각속도는 어느 지점에서나 같으므로 2호기도 같은 각속도를 내야 한다. 조향 0
  으로 두면 2호기가 회전을 막아 1호기가 그 저항까지 이겨야 한다. 힌지형으로
  바뀌면 반드시 되돌릴 것.
- **구현 방향 제안**: 실물 결합 상태에서 (1) `ros2 topic hz
  /fleet/cmd_vel/unit2`가 50 Hz로 잡히는지 — 안 잡히면 QoS/도메인 문제이고
  여기서 막히면 아래는 볼 필요가 없다, (2) `linear_sign`/`angular_sign`이
  맞는지 **낮은 속도부터** 확인 — 부호가 반대면 두 호기가 정면으로 서로를
  민다, (3) `coupled` 모드에서 `drive/arbiter`가 `cmd_vel/coupled`를 정상
  선택하는지 확인한다. 1호기 `/mode`를 `coupled`로 바꾸는 자동 전환이 없어
  웹 콘솔에서 사람이 눌러야 한다는 점도 같이 확인한다. 결합 로직 자체의
  재작성(방위각 정렬)도 함께 실물 검증이 필요하다 — 2번 항목 참고.

## 20. motor 노드 — CAN 인터페이스 순간 두절로 크래시 (조사만, 재현 미확인)

- **필요성**: 2026-09-01 실물 시도 중(14번 항목 참고) `unit2` motor 노드가
  기동 약 8초 만에 죽었다 — 로그: `CAN transmit unavailable; retrying
  with backoff: Failed to transmit: No such device or address [Error
  Code 6]`. `motor.py`가 이 에러에서 재시도만 하다 결국 `Shutting
  down...`으로 스스로 종료한 것으로 보인다(정확한 재시도 한도/종료 조건은
  `motor.py` 코드 확인 필요 — 이번 세션은 로그만 사후 확인했다). motor가
  죽으면 arbiter가 아무리 정상 명령을 보내도 실제 구동이 안 되므로, RC든
  수동이든 어떤 조종 방식이든 이 문제가 있으면 전부 "무반응"으로 보인다
  — 14번 항목의 "RC 무반응" 원인 후보 중 하나이자 RC와 무관하게 그
  자체로도 문제.
  - 문제 발생 직후 사람이 재확인한 `ip -details link show can0`은
    `state UP`/`ERROR-ACTIVE`(정상)였다 — 즉 지금은 인터페이스가 살아
    있다. USB-CAN 동글(`pcan_usb`)의 순간적 접촉 불량이나 커널의 USB
    재열거(다른 프로세스의 USB 점유, 전원 흔들림 등) 추정이지 확정은
    아니다.
- **구현 위치**: `src/base/base/motor.py`(CAN 송신 실패 처리·재시도/종료
  로직), 배선(라즈베리파이 ↔ USB-CAN 동글 ↔ CAN 버스) 자체 점검.
- **구현 방향 제안**:
  1. 다음 실물 시험 때 재현되는지부터 확인 — 한 번뿐이면 배선/커넥터
     흔들림 같은 물리적 우연일 가능성이 크고, 반복되면 동글 자체나
     드라이버 문제로 좁혀야 한다.
  2. `motor.py`가 CAN 송신 실패 시 무한 재시도 대신 바로 죽는 조건이
     뭔지 코드로 확인하고, 필요하면 "일시적 두절은 재시도로 버티고
     완전 두절만 종료"하도록 재시도 한도/백오프를 다듬는 것을 검토한다
     (단, 안전 측면에서 "죽지 않고 계속 재시도"가 항상 옳은지는 사용자
     판단 필요 — 무한 재시도가 오히려 위험 신호를 숨길 수도 있다).
  3. dmesg/journalctl로 USB 재열거 흔적이 있는지 다음 발생 시 같이
     확인한다.

## 21. 웹/RC 조종 방식 — 배타적 전권으로 강화(주행·모드·솔레노이드·컨베이어 전부) (구현됨 — 실물 검증 필요)

- **필요성**: 사용자 요청. 지금 `rc/enabled` 토글은 절반만 배타적이다 —
  - **웹→RC 방향은 이미 완전히 막힌다**: `rc_bridge.py`가 `enabled=False`면
    채널 자체를 무시하므로([rc_bridge.md](docs/fleet/rc_bridge.md)), 웹이
    선택돼 있으면 RC 입력은 아무 효과가 없다.
  - **RC→웹 방향은 모드만 UI에서만 잠그고, 서버는 안 막는다**: RC가 켜져
    있어도 `POST /api/unit/<n>/mode`(웹 콘솔 API를 직접 호출하면)/
    `solenoid`/`roller`가 서버에서 그대로 통과해 `rc_bridge`가 같은
    토픽(`solenoid_cmd`/`roller_manual_cmd`/`drive/mode_cmd`)에 쓰는 값과
    경쟁 상태가 생긴다(`unit.html`의 "모드" 버튼만 disabled로 UI에서
    막아둔 것이지 코드 레벨 중재는 없다는 게 기존 문서에도 명시돼 있음).
    특히 `_tick_drive`는 컨베이어를 **모드와 무관하게 20Hz로 계속
    재발행**하므로, RC가 컨베이어를 조작 중이어도 웹이 마지막으로 쥔 값을
    계속 같이 쏴서 실제로 눈에 보이는 떨림/오작동을 만들 수 있다.
  - 사용자가 원하는 최종 형태: 웹/RC 토글 하나로 **완전한 전권 교대** —
    웹 선택 시 RC 입력은 완전 무시(이미 됨), 웹이 주행/모드 전환/
    솔레노이드/컨베이어 전부 조종 가능. RC 선택 시 웹의 주행/모드 전환/
    솔레노이드/컨베이어 조작이 전부(서버 레벨로) 막히고 RC가 전부 조종.
    RC로 조작하며 바뀌는 값(모드/솔레노이드 잠금 상태/컨베이어 값)은
    계속 웹에 그대로 피드백돼야 한다(이미 `drive/status`·`solenoid_status`
    경유 `coupling/status`·`roller_manual_cmd` 경유 `cargo_load` 상태로
    피드백 경로 자체는 있다 — 발행자 무관하게 실제 상태를 반영하는
    구조라 새 토픽은 필요 없을 것으로 보임, 구현 중 재확인).
- **구현 위치**:
  - `src/mars_console/mars_console/console.py` — `rc/status.enabled`를
    "RC가 지금 전권을 쥐고 있는가"의 단일 판단 근거로 삼는
    `rc_owns(unit)` 같은 헬퍼 추가. `api_mode`/`api_drive`/`api_solenoid`/
    `api_roller`에서 이 값이 참이면 409로 거부. `_tick_drive`의 컨베이어/
    주행 재발행도 이 값이 참이면 건너뛴다(단순 API 거부만으론 이미 쥔
    값이 `input_timeout`까지 계속 재발행될 수 있어서). `estop`/
    `rc/enabled`/`marker/axes`/`mission`은 그대로 예외(안전 정지와 조종
    방식 전환 자체는 항상 되어야 하고, 미션 버튼은 이번 요청 범위 밖).
  - `src/mars_console/mars_console/templates/unit.html` — 이미 있는
    "모드 버튼 잠금" 패턴을 컨베이어 슬라이더/버튼, 솔레노이드 버튼,
    주행 패드/키보드까지 넓힌다. RC 전권 진입 시 웹 쪽 보류 중이던 입력
    (주행 held, 컨베이어 값)은 0으로 리셋해 나중에 웹이 되찾을 때 예전
    값이 갑자기 다시 나가지 않게 한다.
  - `src/fleet/fleet/rc_bridge.py` — 변경 없음(이미 웹→RC 차단은 완전함).
- **구현 방향 제안**: `rc_owns()`는 `feeds[unit]['rc']`의 `enabled` 필드를
  `stale_seconds` 안에서만 신뢰한다 — rc_bridge가 죽어서 `rc/status`가
  끊기면 자동으로 "RC 전권 아님"으로 풀려 웹이 다시 조종할 수 있어야
  한다(락 상태로 영구 고착되면 안 됨, 안전 우선).
- **2026-09-01 구현 경과**: 위 설계대로 `dev/hoyeong/rc_web_exclusive_control`
  브랜치에서 구현했다.
  - `console.py`에 `rc_owns(unit)` 추가(설계대로 `stale_seconds` 안의
    최신 `rc/status.enabled`만 신뢰), `api_mode`/`api_drive`/
    `api_solenoid`/`api_roller`에 409 게이트 추가, `_tick_drive`도
    `rc_owns(unit)`이면 해당 호기의 주행/컨베이어 재발행 자체를 건너뛰게
    수정. `snapshot()`에 `rc_owns` 필드 추가(신선도까지 반영하는 값 —
    `rc_enabled`와 달리 이걸 프런트엔드 잠금 판단에 써야 락이 영구
    고착되지 않는다).
  - `unit.html`에 `rcOwns()` 헬퍼와 `renderLocks()`를 추가해 주행 패드/
    키보드/속도 슬라이더, 컨베이어 슬라이더·버튼, 솔레노이드 버튼을
    모드 버튼과 같은 기준으로 잠근다. RC 전권 진입 전이(false→true) 순간
    `held`/`rollerHeld`를 0으로 리셋한다.
  - 새 테스트 `src/mars_console/test/test_rc_exclusive_control.py`(12개):
    `rc_owns()`의 기본값/fresh/stale/no-rc-unit 경우, `_tick_drive`가
    RC 전권일 때 발행을 건너뛰는지/아닐 때 정상 발행하는지, 4개 API의
    409/estop·rc-enable 예외/호기별 독립(2호기 잠김이 3호기엔 영향 없음)
    까지 검증. `mars_console` 스위트 전체 28개, 저장소 전체(mission/
    mars_console/drive/fleet) 102개 모두 통과(회귀 없음).
  - 문서: [docs/ui/console.md](docs/ui/console.md),
    [docs/fleet/rc_bridge.md](docs/fleet/rc_bridge.md)(있던 "코드 레벨
    우선순위 중재는 없다"는 문장이 이제 stale해져 같이 고침) 갱신.
- **아직 안 된 것 (실물 검증 대상)**: 자동화 테스트만 끝났다. 웹과 RC를
  동시에 조작해봐도 실제로 경쟁 상태가 안 생기는지, RC 전권 중 웹
  화면에서 버튼들이 실제로 잠기고 RC 조작 값(모드/솔레노이드/컨베이어)
  이 화면에 잘 피드백되는지는 다음 실물 시험 때 확인해야 한다. 이 항목이
  검증되면 AGENTS.md 1-1절에 따라 `dev/hoyeong/rc_web_exclusive_control`을
  `dev/hoyeong/main`으로 자동 병합한다.

## 27. `coupling.yaml`의 옛 PID 게인 파라미터가 죽어 있음 — 정리 또는 재사용 여부 결정 필요

- **필요성**: 2026-09-01 `coupling.py` 정렬 로직을 방위각(atan2) 기반 P
  제어로 재작성하면서(위 2번 항목 참고), `_alignment_command()`는
  이제 `bearing_tolerance_deg`/`kp_bearing_rps_per_rad`/
  `align_stable_samples`만 읽는다. 그런데 `coupling.yaml`에는
  `k_lateral`/`ki_lateral`/`kd_lateral`/`k_yaw`/`ki_yaw`/
  `lateral_integral_limit_m_s`/`yaw_integral_limit_deg_s`/`k_roll`/
  `k_pitch`/`lateral_tolerance_m`/`lateral_tolerance_marker_ratio`/
  `roll_tolerance_deg`/`pitch_tolerance_deg`/`yaw_tolerance_deg`가
  여전히 `declare_parameter`만 되고 코드 어디에서도 읽히지 않는다 —
  게다가 `coupling.yaml`에는 이 값들의 산정 근거(실측치, 언제·왜
  이 값으로 정했는지)가 상세한 주석으로 남아 있어, 실제로 튜닝에
  쓰이는 값처럼 보인다. `mission/coupling/status`의
  `pi.lateral_integral_ratio_s`/`pi.lateral_rate`/`pi.yaw_integral_deg_s`
  도 같은 이유로 항상 `0`만 발행하는 죽은 필드다.
- **구현 위치**: `src/mission/mission/common/coupling.py`
  (`_alignment_command`, `_publish_status`의 `pi` 블록),
  `src/mission/config/coupling.yaml`. 발견 경위는
  [docs/mission/coupling.md](docs/mission/coupling.md) "설정" 절의
  "주의" 항목 참고.
- **구현 방향 제안**: 다음 중 하나로 정리한다 — (a) 방위각 P 제어만으로
  실물 시험이 충분히 안정적이면 죽은 파라미터·주석·`pi` 상태 필드를
  코드/yaml에서 걷어낸다, (b) 실물에서 P 제어만으로는 정상상태 오차나
  진동이 남으면 그때 I/D 항을 방위각 기준으로 다시 설계해 살려 쓴다.
  둘 중 어느 쪽이든 결정 전에는 이 파라미터들을 건드려도 아무 효과가
  없다는 점을 다음에 튜닝할 사람이 먼저 알아야 한다.

## 28. `verify_pull` 실패 시 솔레노이드를 풀어 버림 — 해제하지 말고 재결합해야 함

- **필요성**: 결합 검증(`verify_pull`)은 후진하며 전류가 기준 이상으로
  올라오는지 보는 단계다. 여기서 `verify_timeout_s`(1.0초) 안에 전류가
  안 오르면 지금은 `_abort('verify_timeout', unlock=True)` 로 빠져
  **솔레노이드를 해제**하고 `error` 로 멈춘다. 사용자 확정(2026-09-03):
  솔레노이드는 기계적으로 잠기므로 자동으로 풀지 않는다 — 검증이
  실패했다는 것은 "덜 붙었다"는 뜻이니 **풀지 말고 결합을 한 번 더
  돌려야** 한다. `solenoid_unlocked`/`verify_current_lost` 경로도 같은
  이유로 `unlock=True` 다.
- **구현 위치**: `src/mission/mission/common/coupling.py` 의
  `verify_pull` 분기와 `_abort(..., unlock=True)` 호출부.
- **구현 방향 제안**: 검증 실패는 `error` 가 아니라 `align` 부터
  재시도하는 경로로 보낸다(재시도 횟수 상한을 두어 무한 반복을 막는다).
  솔레노이드는 건드리지 않는다 — 해제는 사람이 명시적으로 누를 때만.

## 29. 결합 완료(`locked`) 후 주행 모드가 자동으로 `coupled` 로 안 넘어감

- **필요성**: 대회는 수동 주행과 자동 주행 구간이 나뉘어 있고, 자동
  주행 중에는 사람이 개입할 수 없다. 그런데 지금은 결합이 `locked` 가
  돼도 `drive/arbiter` 의 모드가 그대로라, 사람이 `drive/mode_cmd` 로
  `coupled` 를 쏘기 전에는 협조 주행이 시작되지 않는다. fleet
  `ACTIONS` 에도 `coupled` 가 없어 1호기가 대신 시킬 수도 없다 —
  자동 주행 구간에서 결합 후 그대로 멈춰 선다.
- **구현 위치**: `src/drive/drive/arbiter.py` (`_tick`, `_phase`).
- **구현 방향 제안**: arbiter 가 `coupling_status['phase'] == 'locked'`
  를 보면 스스로 모드를 `coupled` 로 올린다. Fleet Protocol 은 고정
  계약이라 건드리지 않는다(2호기만 고치면 닫힌다). 수동 주행 중에
  멋대로 올라가지 않도록 승격 대상 모드를 제한한다.

## 30. 결합 해제 전용 키 분리 — 지금은 `취소`와 겸용

- **필요성**: 콘솔에 `결합 취소 · 해제` 버튼 하나가
  `/api/coupling/cancel` 로 취소와 솔레노이드 해제를 함께 한다.
  사용자 확정(2026-09-03): 결합 해제는 자동으로 하지 않고 **사람이
  명시적으로 누를 때만** 한다. 그러려면 "진행 중인 결합을 멈추는 것"과
  "잠긴 솔레노이드를 푸는 것"이 서로 다른 키여야 한다.
- **구현 위치**: `src/drive/drive/test_console.py` 와
  `src/drive/drive/templates/test_console.html`,
  `src/mission/mission/common/coupling.py` 의 `_on_command`
  (`'cancel'` 과 `'unlock'` 이 지금은 같은 분기다).
- **구현 방향 제안**: `unlock` 을 `cancel` 에서 떼어내 솔레노이드
  해제만 하는 경로로 만들고, 콘솔에 별도 버튼을 둔다. 릴레이 과열
  방지 타이머는 이미 있다(커밋 `1d45225`).

## 31. 결합 라우팅이 여름 노드(`cargo_load`) 안에 있음 — 공용으로 분리 필요

- **필요성**: 1호기의 `couple` 액션을 `coupling/cmd` 로 넘기는 코드가
  `src/mission/mission/summer/cargo_load.py` 에 있다. 결합은 겨울
  구간에서도 쓰는데(사용자 확정), 지금은 여름 노드를 거쳐야만 한다.
  나중에 계절별로 노드를 껐다 켜는 구조가 되면 겨울 결합이 끊긴다.
- **구현 위치**: `src/mission/mission/summer/cargo_load.py`
  (`action == 'couple'` 분기), `src/mission/mission/common/`.
- **구현 방향 제안**: `fleet/action` 의 `couple` 을 받아
  `coupling/cmd` 로 넘기는 일을 `coupling.py` 자신이 하게 한다
  (`coupling` 은 지금 `fleet/action` 을 구독하지 않는다). 그러면
  중간 노드 없이 여름·겨울 모두에서 동작하고 `cargo_load` 는 화물만
  담당하게 된다.

## 32. `yolo_manager` 제거 — 2호기는 YOLO 를 쓰지 않음

- **필요성**: `src/mission/mission/common/yolo_manager.py`(174줄)와
  `src/mission/config/yolo_models.yaml` 이 남아 있지만 어떤 launch
  에서도 띄우지 않는다. 사용자 확정(2026-09-03): 2호기는 YOLO 를
  쓰지 않는다. 남겨 두면 `model_root` 경로 문제(옛 TODO 5번)처럼
  계속 점검 대상으로 끌려다닌다.
- **구현 위치**: `src/mission/mission/common/yolo_manager.py`,
  `src/mission/config/yolo_models.yaml`, `src/mission/setup.py`
  진입점, 이를 참조하는 문서.
- **구현 방향 제안**: 파일과 진입점, 문서 참조를 함께 걷어낸다.
  옛 TODO 4·5번(yolo 재배선 여부, `model_root` 경로)은 이 항목으로
  대체해 닫는다.

## 33. `follow_leader` yaw 항 활성화 (`k_yaw`) — 부호 실물 확인 필요

- **필요성**: 1호기가 회전하면 2호기도 그에 맞춰 정면을 돌려야
  하는데(사용자 확정 2026-09-03), `follow.yaml` 의 `k_yaw` 가 `0.0`
  이라 정면 정렬 항이 꺼져 있다. 지금은 좌우 오프셋만 줄이므로
  1호기가 제자리 회전하면 2호기는 따라 돌지 않는다.
- **구현 위치**: `src/mission/config/follow.yaml` (`k_yaw`),
  `src/mission/mission/winter/follow_leader.py` (`_combine_angular`).
- **구현 방향 제안**: `angular.z = -k_angular*lateral + k_yaw*yaw` 이고
  상한이 0.5 rad/s 다. yaw 20도(0.35rad)에서 0.07 rad/s 가 나오도록
  `k_yaw: 0.2` 부터 시작한다. **부호가 미검증이다** — 코드가
  `+k_yaw*yaw` 인데 yaw 부호 규약을 실물로 확인한 적이 없어, 반대면
  추종이 더 나빠진다. 낮은 값으로 1호기를 좌우로 돌려 보며 부호부터
  가린다. 무한궤도는 조향 입력이 하나라 좌우항과 yaw 항이 서로
  밀어낼 수 있다(결합 정렬에서 그래서 yaw 를 뺐다 — 27번 항목) —
  진동이 생기면 `k_yaw` 를 먼저 줄인다.
