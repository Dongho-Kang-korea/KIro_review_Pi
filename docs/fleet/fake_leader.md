# fake_leader — `src/fleet/fleet/fake_leader.py`

**최신화**: 2026-08-29

## 역할

1호기 없이 2호기 단독으로 **추종(follow) 모드를 벤치에서 시험**하기
위한 가짜 리더. `/fleet/state`를 대신 주기적으로 발행해
[follower](follower.md)의 자동 `follow` 전환(`_on_state`)과
`drive/arbiter`의 `fleet_alive()` 판정을 실물 1호기 없이도 통과시킨다.

노드/모듈이 아니라 **시험 전용 도구**다 — `robot.launch.py`(실제 운용
런치)에는 포함돼 있지 않고, [web_manual.launch.py](../../RUNBOOK.md)에만
`start_fake_leader`(기본 true)로 들어간다.

## 실행

`mars_launch/web_manual.launch.py`에서 `start_fake_leader:=true`(기본값)로
뜬다. 단독 실행:

```bash
ros2 run fleet fake_leader --ros-args -p section:=spring
```

## 설정

노드 파라미터로만 받는다(yaml 없음) — `section`(기본 `spring`,
`fleet/protocol.py`의 `SECTIONS` 중 하나), `mode`(기본 `auto`),
`rate_hz`(기본 5.0, `protocol.STATUS_RATE_HZ`와 동일하게 맞춤).

`section`이 `summer`가 아니면 [follower](follower.md)가 개별 명령 없이
자동으로 `follow` 액션을 건다 — summer 화물 적재 시나리오를 시험할 게
아니면 기본값(`spring`)을 그대로 쓰면 된다.

## 토픽

- 발행: `/fleet/state`(절대경로, `protocol.TOPIC_STATE`) — `section`/
  `mode`/`leader_stopped: false`/`source: 'fake_leader'`를 `rate_hz`로
  반복 발행.

## 동작 요약

- 딱 하나만 한다: `/fleet/state`를 계속 쏜다. 명령(`/fleet/cmd/unit{N}`)은
  안 보낸다 — summer가 아닌 계절은 `/fleet/state`만으로 follower가 자동
  `follow`를 걸기 때문에 이걸로 충분하다. summer 화물 적재처럼 개별
  명령이 필요한 시나리오는 이 노드로 커버되지 않는다(콘솔의 "미션 명령"
  패널이나 별도 발행이 필요).
- 시작 시 `get_logger().warn()`으로 "가짜 리더 켜짐" 경고를 남긴다 —
  실물 1호기와 헷갈리지 않게 로그에서 바로 보이게 하기 위함이다.

## 연결

- 위: 없음(사람이 launch 인자로 켜고 끔).
- 아래: `fleet/follower`(`/fleet/state` 구독), `drive/arbiter`(같은 토픽
  구독, `fleet_alive()`).

## 구현 상태

완성. 단위 검증(2026-08-29): `Follower._on_state()`에 이 노드가 실제로
보내는 형식의 메시지를 주입해 `action`이 `follow`로 바뀌는 것을 확인했다
(합성 테스트, 실물 launch 재시작 후 웹 콘솔에서 "추종" 버튼이 실제로
풀리는지는 [RUNBOOK.md](../../RUNBOOK.md) 5절로 확인 필요).

## 알려진 이슈

**실물 1호기와 같은 `ROS_DOMAIN_ID`에서 절대 같이 켜면 안 된다** —
두 발행자가 동시에 `/fleet/state`를 쏘면 follower/arbiter가 어느 쪽
값을 쓰는지 예측할 수 없다. `web_manual.launch.py`의 기본
`test_domain`(77)이 실물 운용 도메인(10)과 분리돼 있는 한 정상적인
사용에서는 충돌하지 않는다 — 도메인을 직접 바꿔 쓸 때만 주의하면 된다.
