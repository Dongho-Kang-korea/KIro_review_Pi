# KIro_review_Pi

MARS 2호기 — 2026 육군참모총장배 국방로봇경진대회 대회 종료 시점 코드 스냅샷.

> 팀 운영 정책 문서(`AGENTS.md`)와 개발용 측정 도구(`test_basic/`)는 이 저장소에 포함하지 않았다.
> 문서와 `TODO.md` 에 남아 있는 해당 언급은 대회 당시의 기록이다.

**최신화**: 2026-08-31

2호기 전용 ROS 2 워크스페이스다. 3호기는 설계에서 빠졌다(TODO 19).

노드별 상세 문서는 [docs/](docs/README.md), 배선/
실측은 [HARDWARE.md](HARDWARE.md), 실행 절차는 [RUNBOOK.md](RUNBOOK.md),
미구현 항목은 [TODO.md](TODO.md)를 참고한다.

## 패키지

- `base`: CAN 모터/롤러/전류, 카메라/ArUco 마커, UWB, BNO086, 솔레노이드
- `fleet`: Fleet Protocol 명령 수신, 중복 `seq` 차단, watchdog, 5 Hz 상태
- `mission`: 마커+UWB 추종, 여름 물자 적재, 마커 자동 결합
- `drive`: 모드 소유 + 최종 속도 중재, 키보드 시험 입력, 로컬 시험 화면
- `mars_launch`: ROS 기본 `launch` 패키지와 충돌을 피한 실행 조립

## 네임스페이스 — 먼저 읽을 것

모든 노드가 `/unit2` 아래에 뜬다. **1호기와 토픽 이름이 겹치기
때문이다.** 양쪽 다 `cmd_vel` / `roller_cmd` / `motor/status` / `solenoid_cmd` 를
쓰므로, 같은 `ROS_DOMAIN_ID` 에 올라오면 1호기 주행 명령이 2호기 모터로 그대로
들어간다.

그래서 노드 코드의 내부 토픽은 전부 **상대 이름**이다.

```python
self.create_publisher(Twist, 'cmd_vel', 10)      # 맞다 -> /unit2/cmd_vel
self.create_publisher(Twist, '/cmd_vel', 10)     # 틀렸다 -> 루트로 올라가 충돌
```

예외는 호기 간 규약 토픽 `/fleet/state`, `/fleet/cargo/target`,
`/fleet/cmd/unitN`, `/fleet/status/unitN` 넷뿐이다. 1호기와 공유해야 하므로
절대 경로를 유지한다.

파라미터 yaml 키도 `/**/motor:` 형태다. `motor:` 로 두면 `/unit2/motor` 와
매칭되지 않고, **에러 없이 조용히 무시된 채 코드 기본값이 쓰인다** (모터 ID까지
포함해서). 새 노드를 추가할 때 이 형식을 지킬 것.

## 모드

`drive/mode_cmd` 로 런타임 전환한다. 시작은 항상 `idle` 이고 운용 UI 가 올린다.

| 모드 | 주행 소스 | 미션 명령 출처 | 1호기 필요 |
|---|---|---|---|
| `idle` | 없음 (0) | 없음 | 아니오 |
| `manual` | `cmd_vel/manual` | 없음 (인식 노드는 계속 돎) | 아니오 |
| `auto` | 미션 노드 | `mission/request` (UI) | 아니오 |
| `follow` | `cmd_vel/follow` | `/fleet/cmd/unitN` (1호기) | **예** |
| `coupled` | `cmd_vel/coupled` | 없음 (1호기 속도 `/fleet/cmd_vel/unit2` 복제만) | **예** |

`cargo_load` 와 `follow_leader` 는 `mission/action` 만 구독한다. arbiter 가 모드에
따라 권위 있는 명령 하나를 골라 거기로 재발행한다 — 예전처럼 `fleet/action` 을
직접 보면 1호기가 없을 때 아무 미션도 시작할 수 없다.

`mission/request` 는 "해달라는 요청", `mission/action` 은 "확정된 명령"이다.

추종은 `/fleet/state` 가 2.5초 안에 수신 중일 때만 진입할 수 있고, 진입 후
끊기면 스스로 `idle` 로 내려온다.

## 계절

**여름에만 자체 미션(화물 적재)이 있고 나머지 구간은 전부 추종이다.** 그래서
계절별로 띄우는 노드가 갈리지 않는다 — `cargo_load` 는 항상 올려두고 명령이
없으면 가만히 있는다. `season` 인자는 UI 표시용이다.

## 실행

```bash
cd <워크스페이스 디렉터리>
colcon build --symlink-install
source install/setup.bash

# 본체
ros2 launch mars_launch robot.launch.py

# 본체 + 호기 로컬 시험 화면 :5000
ros2 launch mars_launch test_drive.launch.py

# 정비용 단독 수동 — 모터와 중재기만. UI도 1호기도 필요 없다
ros2 launch mars_launch manual.launch.py
```

수동 조작 노드는 터미널(TTY)이 필요해 launch 에 넣지 않았다. 다른 창에서:

```bash
export ROS_DOMAIN_ID=10
ros2 run drive manual --ros-args -r __ns:=/unit2
```

## 미장착·미보정 하드웨어

UWB는 `base/config/uwb.yaml`에서 `enabled: true`로 활성화됐지만(2026-08-29
실측 확인) 정밀 오프셋 보정은 아직이다. BNO086은 `base/config/imu.yaml`에서
`enabled: true`, I2C 주소 `0x4B`로 실측 확인된 상태다(heading 영점
`heading_offset_deg`는 아직 미교정). 화물 잠금은 별도 하드웨어(서보 등) 없이
`hold` 액션에서 `loaded` 단계로 바로 완료 처리한다(서보는 쓰지 않기로 결정,
[TODO.md](TODO.md) 참고).

여름 적재는 세 곳에서 하드웨어를 기다린다.

| 단계 | 필요한 것 |
|---|---|
| `approach` | UWB (활성화됨, 오프셋 정밀 보정 남음) |
| `search` → `turning` | BNO086 (장착됨, heading 영점만 남음) |
| `backing` → `contact` | `motor.yaml` 의 `contact_current_threshold_a` 실측값 |

접촉 전류 기준은 적재와 자동 결합이 **하나를 공유한다**. 실측 전에는 `0.0` 으로
두어 자동 접촉 판정이 임의로 진행되지 않게 한다.

추종은 UWB 없이도 돈다. `mission/config/follow.yaml` 의
`allow_marker_distance_fallback: true` 가 마커 거리를 폴백으로 쓰는
설계다. `marker/pose` 는 나오고(ArUco 캘리브레이션 확정·실측 검증
통과 — [marker_vision 문서](docs/base/marker_vision.md) 참고),
`follow_leader.py` 는 이 메시지를 타입 그대로 구독해서 필드명 문제가
없었다 — UWB 없이도 폴백이 동작한다. UWB 를 달면 그쪽이 우선한다.

## 자동 결합

`marker/pose` 가 나오고 `coupling.py` 도 새 필드명(`x_mm`/`y_mm`/
`z_mm`/`roll_deg`/`pitch_deg`/`yaw_deg`)에 맞게 재배선됐다(자동화
테스트 통과, `src/mission/test/`). 다만 `contact_current_threshold_a`
실측 전에는 `_eligibility()`가 `align` 진입 자체를 막으므로,
실물로 끝까지(`contact`→`locked`) 시험하려면 [TODO.md](TODO.md)의
해당 항목이 먼저 끝나야 한다. 아래 흐름은 설계상 동작이다.

시동 후 마커가 1초 연속 인식되면 당시 solvePnP 자세(x/y/z/roll/pitch/yaw)를
기준으로 저장한다. 시험 화면에서 시작하면 기준 정렬, 저속 전진, 전류 접촉 확인,
솔레노이드 잠금 순으로 진행한다. 버튼은 인식이 잠깐 끊겨도 1초간 유지하지만,
결합 주행 중 마커가 0.35초 끊기면 즉시 정지한다.

기준 대비 거리(z) 오차의 선형 속도는 이전 결합기의 PID 값과 적분 제한·
미분 필터를 쓰며, 게인은 `mission/config/coupling.yaml` 에서 조정한다.

ArUco 마커 설정(딕셔너리·허용 ID·크기)은 `base/config/marker.yaml`이 최신
출처다. CAN 응답과 전류가 0.5초 이상 끊기면 접촉 판정을 중단하며, 최종 속도 명령도
0.5초 이상 끊기면 모터 노드가 자체적으로 정지한다. 롤러도 마찬가지다.
