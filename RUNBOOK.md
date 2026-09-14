# 실행 절차

**최신화**: 2026-09-03

빌드부터 각 launch 조합, 확인해야 할 기대값까지. 노드별 세부는
[docs/](docs/README.md), 배선/실측은 [HARDWARE.md](HARDWARE.md)를 본다.
이 문서는 [AGENTS.md](AGENTS.md) 2절 정책에 따라
`src/mars_launch/launch/*.launch.py`나 실행 절차에 영향 있는 변경이 생기면
같이 갱신한다.

## 0. 빌드

```bash
cd <워크스페이스 디렉터리>   # AGENTS.md 1-1절 표: KIRO_rasp_ws_main/_hoyeong/_dongho 중 지금 작업 중인 곳
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install`을 쓰면 `src/**/config/*.yaml`을 고친 뒤 다시 빌드하지
않아도 바로 반영된다(파이썬 파일도 동일).

### 자동화 테스트 (하드웨어 불필요)

```bash
cd <워크스페이스 디렉터리>   # 위와 동일
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/mission/test/ src/mars_console/test/ src/drive/test/ src/fleet/test/ -v
```

`mission` 패키지의 순수 로직(마커 pose 필드 파싱, 정렬 오차 계산, EMA
스무딩, 수동 컨베이어 병합 등)과 `mars_console`(모드 동기화, 솔레노이드/
컨베이어 수동 조작 게이팅), `drive`(arbiter의 rc 모드/소스 판정),
`fleet`(rc_bridge의 RC 채널 정규화)을 실물 없이 검증한다. 관련 코드를
고칠 때는 테스트도 같이 갱신한다.

## 1. 본체 단독 (2호기)

```bash
ros2 launch mars_launch robot.launch.py                  # 2호기, summer
```

기대값:
- `ros2 topic echo /unit2/motor/status`에 `can_interface_open`이
  실물 CAN 연결 시 `true`(없으면 `false`, 노드는 정상 기동).
- `ros2 topic echo /unit2/drive/status`의 `mode`가 `idle`로 시작.
- 운용 UI(`mars_console`, 별도 PC) 없이도 노드 자체는 전부 뜬다 — UI가
  `drive/mode_cmd`를 보내야 실제로 움직인다.

## 2. 본체 + 호기 로컬 시험 화면 (:5000)

```bash
ros2 launch mars_launch test_drive.launch.py
```

`http://<호기IP>:5000`에서 카메라·상태·비상 정지·자동 결합 시작/취소/해제를
확인한다([test_console.md](docs/drive/test_console.md)).

## 3. 정비용 단독 수동 (UI·1호기 불필요)

```bash
ros2 launch mars_launch manual.launch.py
# 카메라+시험 화면도 필요하면:
ros2 launch mars_launch manual.launch.py console:=true
```

다른 창에서 조작 노드를 띄운다(TTY 필요, launch에 안 들어 있음):

```bash
export ROS_DOMAIN_ID=10
ros2 run drive manual --ros-args -r __ns:=/unit2
```

기대값: `w/a/s/d`로 `/unit2/cmd_vel`이 움직인다. `q` 또는 조작 노드를
죽이면 0.5초(`source_timeout_s`) 안에 arbiter가 스스로 0을 낸다.

## 4. Domain 격리 수동 주행 시험 (실전과 분리)

```bash
ros2 launch mars_launch manual_drive.launch.py
```

`ROS_DOMAIN_ID`를 `test_domain`(기본 77)으로 덮어써서 실전(10)과 격리한다.
모터 단독 시험에 쓴다. `start_manual:=false`/`start_motor:=false`로 개별
끌 수 있다.

## 5. 웹 수동조종 시험 (콘솔 + 모터 + IMU + 카메라 + 솔레노이드/컨베이어 + 추종용 노드 일괄)

```bash
ros2 launch mars_launch web_manual.launch.py
```

`http://localhost:5100`(mars_console)에서 `/unit/2`로 들어가 수동 조작.
`test_domain`(기본 77)으로 격리된다. `start_motor`/`start_imu`/`start_camera`/
`start_fleet`/`start_fake_leader`(전부 기본 true)로 개별 끌 수 있다.
`base/solenoid`, `mission/cargo_load`는 게이트 없이 항상 뜬다.

**이 런치는 1호기 없이 2호기 단독으로 주행·추종·컨베이어·솔레노이드·
ArUco 인식을 전부 시험할 수 있게 만든 벤치 시험 전용 런치다**
(`fleet/fake_leader`가 `/fleet/state`를 대신 흉내 낸다 — 실물 1호기와
같은 `ROS_DOMAIN_ID`에서는 `start_fake_leader:=false`로 반드시 끌 것,
[fake_leader 문서](docs/fleet/fake_leader.md) 참고).

기대값:
- 페이지 열자마자 "수동" 버튼이 이미 활성화(모드 경고 안 뜸) — arbiter가
  `start_mode: manual`로 뜨기 때문. 방향 버튼/`wasd`를 누르면 **바로**
  움직여야 한다(2026-08-28 전에는 콘솔 내부 모드 동기화 버그로 안 먹혔음).
- 카메라 화면에 영상이 뜬다(`start_camera:=false`로 끄지 않은 한).
- "솔레노이드 수동 조작" 패널에서 잠금/해제 버튼을 누르면 바로 반영된다
  (상태 행 "솔레노이드"가 잠김/풀림으로 바뀐다).
- "컨베이어(롤러) 수동 조작" 패널에서 정/역방향 버튼을 누르면 롤러가
  돈다(상태 행 "컨베이어"가 "수동 값"으로 표시). "정지"를 누르거나
  페이지를 벗어나면 멈춘다.
- **추종 모드**(2026-08-29부터): "추종" 버튼이 1호기 없이도 바로
  풀려야 한다(`start_fake_leader`가 기본 켜져 있으므로). 눌러서 전환하면
  마커(ID 1)를 카메라 앞에 두고 로봇이 목표 거리(기본 600mm)를 향해
  실제로 움직이는지 확인한다 — `allow_marker_distance_fallback: true`라
  UWB 없이도 ArUco 거리만으로 동작해야 한다. 상태 표의 "1호기 링크"가
  "수신 중"으로 뜨면 fake_leader가 정상 동작 중인 것이다(실제 로그에는
  "가짜(FAKE) 1호기" 경고가 남는다 — 실물 1호기로 착각하지 말 것). 조향은
  거리(P)·좌우 오프셋(P) 두 항이 기본이고, **yaw 정면 정렬 항(`k_yaw`,
  `follow.yaml`)은 기본 0.0(비활성)** — 마커(리더)를 손으로 천천히
  돌려보며 로봇이 그쪽으로 정렬되게 튜닝하려면 `follow.yaml`의 `k_yaw`를
  작은 값부터 올려본다([follow_leader 문서](docs/mission/follow_leader.md),
  [TODO.md](TODO.md) 11번 참고).
- **ArUco 3축 좌표계 표시**(2026-08-29부터): 카메라 패널의 "3축 좌표계
  표시" 버튼을 누르면 마커 위에 X/Y/Z 축이 그려지고, 상태 표에
  `X/Y/Z(mm)`·`Roll/Pitch/Yaw` 수치가 같이 뜬다. 다시 누르면 꺼지고
  수치도 사라진다(pose 미보정 상태면 버튼을 눌러도 아무것도 안 그려짐 —
  정상).

## 6. 운용 콘솔 단독 실행 (운용 PC에서)

로봇이 아니라 **운용 PC**에서, 로봇과 같은 망·같은 `ROS_DOMAIN_ID=10`으로:

```bash
export ROS_DOMAIN_ID=10
ros2 run mars_console console            # http://localhost:5100
```

1·2호기를 한 화면에서 선택해 모드 전환·수동 조작·비상 정지·미션 명령을
보낸다([console.md](docs/ui/console.md)).

## 안전 확인 체크리스트 (실물 시험 전)

- [ ] 접촉 전류 기준(`contact_current_threshold_a`)은 미션마다 따로다
      ([HARDWARE.md](HARDWARE.md) "접촉 전류 기준값" 참고). **결합
      (`mission/config/coupling.yaml`)은 2026-09-01부터 1.7A로 확정돼
      있어 `align`→`contact` 자동 접촉 판정이 실제로 동작한다** — 결합
      시험 전에는 이 값이 지금 로봇의 실제 접근 속도(`contact_speed_mps`,
      기본 0.10 m/s)와 짝이 맞는지 반드시 확인한다. 화물 적재 쪽은
      1호기 config가 아직 `0.0`(미실측)이라 자동 접촉 판정이 시작되지
      않는다(의도된 안전 상태) — `base/config/motor.yaml`의 공통 폴백도
      `0.0` 그대로다.
- [ ] 비상 정지(`safety/emergency_stop`)가 실제로 모터를 멈추는지 시험
      화면(:5000) 또는 `mars_console`에서 먼저 확인한다.
- [ ] `cmd_vel`/`roller_cmd` 발행을 끊었을 때(`source_timeout_s`/
      `cmd_vel_timeout_s`) 모터가 스스로 정지하는지 확인한다.
- [ ] **솔레노이드 극성이 2026-09-01에 네 번째로 재정정됐다**
      (`active_low: false`) — 결합 시험 전에 `solenoid_cmd`로 잠금/해제를
      직접 보내 실제 걸쇠 동작이 기대와 맞는지, 장시간 idle 상태에서
      코일이 뜨거워지지 않는지 다시 확인한다([solenoid](docs/base/solenoid.md)
      "알려진 이슈" 4번 참고).

## 이 문서의 갱신 규칙

`src/mars_launch/launch/*.launch.py`가 바뀌거나 실행 절차/기대값이
바뀌면 이 문서를 같이 고친다 — [AGENTS.md](AGENTS.md) 2절 참고.
