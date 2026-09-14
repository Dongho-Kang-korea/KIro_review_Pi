# launch — `src/mars_launch/launch/*.launch.py`

**최신화**: 2026-09-03

`mars_launch` 패키지는 ROS 기본 `launch` 패키지와 이름이 충돌하지 않도록
분리된 실행 조립 패키지다. 노드 코드는 없고 launch 파일 5종만 있다.

## robot.launch.py — 본체(2호기)

```
ros2 launch mars_launch robot.launch.py                  # 2호기
ros2 launch mars_launch robot.launch.py season:=escort   # 계절만 다르게
```

- 인자: `unit`(2 또는 3, 기본 2 — 네임스페이스가 됨), `season`(spring/
  summer/autumn/winter/escort, 기본 summer — UI 표시/기록용, 뜨는 노드는
  안 갈림).
- `ROS_DOMAIN_ID=10`을 강제 설정한다(1·2호기가 서로 보이려면 필수).
- 모든 노드를 `namespace=unit{N}` 아래 띄운다.
- 띄우는 노드: `base`(motor, camera, marker_vision, imu, uwb,
  solenoid) 전부 + `fleet/follower` + `fleet/rc_bridge`(둘 다 1호기 통신,
  없어도 manual/auto는 그대로 동작 — rc_bridge는 2026-08-30 추가, 토픽/
  채널 매핑 임시값. [rc_bridge 문서](../fleet/rc_bridge.md)) +
  `mission`(follow_leader, cargo_load, coupling, coupled_drive — 계절
  무관 항상, `coupled_drive`는 2026-09-01 추가)
  + `drive/arbiter`(모드 소유자 겸 안전 게이트).
- 노드 추가/삭제가 필요할 때만 이 파일을 고치고, 수치 튜닝은 각 config
  yaml에서 한다.

## manual.launch.py — 정비용 단독 수동

```
ros2 launch mars_launch manual.launch.py
ros2 launch mars_launch manual.launch.py console:=true   # 카메라+시험화면 추가
```

- 올리는 것은 **모터와 arbiter(`start_mode: manual`)뿐** — fleet follower와
  mission 노드를 안 띄운다: follower가 없으면 `safety/stop`이 걸릴 일이
  없고(follower는 종료 시 stop=true를 남기므로 단독 주행에는 방해),
  mission이 없으면 UWB·IMU·전류 기준 미장착 게이트를 안 탄다.
- 조작 노드(`drive/manual`)는 TTY가 필요해 이 launch에 안 들어 있다 —
  별도 창에서 `ros2 run drive manual --ros-args -r __ns:=/unit2`.

## test_drive.launch.py — 본체 + 호기 로컬 시험 화면

```
ros2 launch mars_launch test_drive.launch.py
```

- `robot.launch.py`를 `IncludeLaunchDescription`으로 그대로 포함하고,
  `drive/test_console`(:5000)을 추가한다.

## manual_drive.launch.py — Domain 격리 수동 주행 시험

- `ROS_DOMAIN_ID`를 `test_domain`(기본 77)으로 **덮어써서** 실전
  Domain(10)과 격리한다 — 다른 호기/1호기에 영향을 주지 않고 모터만
  단독으로 시험할 때 쓴다.
- `start_manual`(기본 true)로 `drive/manual`(emulate_tty로 TTY 확보)을,
  `start_motor`(기본 true)로 `base/motor`를 조건부 포함한다.
- `namespace` 인자(기본 unit2)로 네임스페이스를 바꿀 수 있다.

## web_manual.launch.py — 웹 수동조종 시험 일괄 실행

- `test_domain`(기본 77)으로 격리, `arbiter`(`start_mode: manual`) +
  `mars_console`(운용 PC용 콘솔을 **로봇 쪽에서** 같이 띄움, 5100) +
  `start_imu`/`start_motor`(둘 다 기본 true) 조건부 포함.
- `start_fleet`(기본 true)로 `fleet/follower`+`mission/follow_leader`
  +`fleet/rc_bridge`(2026-08-30 추가)를 함께 띄운다 — 벤치에는 1호기가
  없어 RC 채널 데이터는 안 오지만, 콘솔의 "조종기" 모드 버튼 노출은
  확인할 수 있다.
- 웹 입력은 `/unit2/cmd_vel/manual` → arbiter → `/unit2/cmd_vel` → motor/CAN
  경로로 전달된다.

## 구현 상태

완성.

## 알려진 이슈

없음 — 발견되면 [TODO.md](../../TODO.md)에 등록한다.
