# MARS 운용 콘솔

호기 선택 → 호기별 화면. **로봇이 아니라 운용 PC 에서 띄운다.**

ROS 2 Jazzy 가 깔린 리눅스 PC 를 로봇과 같은 망, 같은 `ROS_DOMAIN_ID=10` 에
두면 된다.

## 화면

| 경로 | 내용 |
|---|---|
| `/` | 호기 선택 — 1·2·3호기, 각자 온라인 여부 |
| `/unit/1` | 1호기 — 수동 / 자동 |
| `/unit/2` | 2호기 — 수동 / 자동 / 추종 |
| `/unit/3` | 3호기 — 같은 화면 (아직 미배치라 '응답 없음') |

## 빌드와 실행

```bash
mkdir -p ~/mars_ui_ws && cp -r src ~/mars_ui_ws/
```

```bash
cd ~/mars_ui_ws && colcon build --symlink-install && source install/setup.bash
```

```bash
export ROS_DOMAIN_ID=10 && ros2 run mars_console console
```

`http://localhost:5100` 으로 접속한다. 설정을 바꾸려면:

```bash
ros2 run mars_console console --ros-args --params-file src/mars_console/config/console.yaml
```

## 호기마다 다른 것을 콘솔이 흡수한다

1호기는 토픽이 루트에 있고 2·3호기는 `/unitN` 네임스페이스 아래 있다. 모드를
거는 방법도 다르다 — 1호기는 예전 `/mode` 로 중재 소스를 직접 고르고(계절에
따라 `comp`/`escort`), 2·3호기는 `drive/mode_cmd` 로 `manual`/`auto`/`follow` 를
고른다.

그 차이는 `console.py` 상단의 `UNITS` 표에만 있다. **로봇 쪽 토픽 이름을 바꾸면
이 표를 함께 고쳐야 한다.** 안 고치면 해당 표시가 조용히 빈다.

## 수동 주행

화면 버튼과 키보드(`W`/`A`/`S`/`D`, 방향키) 둘 다 된다. 누르고 있는 동안만
움직인다.

**입력 감시**가 있어서 브라우저가 `input_timeout`(0.5초) 안에 조작을 보내지
않으면 0 을 발행한다. 창을 닫거나 Wi-Fi 가 끊겼을 때 마지막 속도로 계속 가는
것을 막는다.

콘솔이 내는 속도는 `max_linear` / `max_angular` 로 한 번 더 깎는다. 호기 쪽
상한과 별개다.

## 추종 버튼이 잠기는 이유

2·3호기 arbiter 가 내는 `drive/status` 의 `fleet_link` 가 `false` 면 잠근다.
1호기 `/fleet/state` 가 2.5초 안에 안 들어왔다는 뜻이다. 확인할 것:

1. 1호기 `leader` 노드가 떠 있는가 (`robot.launch.py` 의 `enable_fleet`)
2. **1호기가 `ROS_DOMAIN_ID=10` 인가** — 이게 가장 흔한 원인이다
3. 두 기계가 같은 망에 있는가

`leader` 는 계절 미션 없이도 `/fleet/state` 를 1Hz 로 내보내므로, 1호기가
`robot.launch.py` 만 떠 있어도 조건은 충족된다.

## 아직 안 해본 것

**이 코드는 실행된 적이 없다.** 문법 검사와 토픽 이름 대조만 했다. 첫 기동에서
호기가 '응답 없음' 으로 뜨면 다음을 확인할 것:

```bash
export ROS_DOMAIN_ID=10 && ros2 topic list | grep unit2
```

`/unit2/motor/status` 가 보여야 한다. 안 보이면 도메인이나 네임스페이스 문제다.
