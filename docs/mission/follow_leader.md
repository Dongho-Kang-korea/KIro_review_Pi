# follow_leader — `src/mission/mission/winter/follow_leader.py`

**최신화**: 2026-09-03 (`mission/common` → `mission/winter` 패키지 이동 반영)

## 역할

마커 자세와 UWB 거리로 1호기를 추종하는 속도 명령을 만든다. 여름을
포함한 **모든 계절에서 공용으로 쓰인다**(여름만 `cargo_load`가 추가로
붙는다). `mission/action`이 `follow`일 때만 실제로 움직인다.

**패키지 위치 주의(2026-09-01)**: 파이썬 패키지는 `mission.common`에서
`mission.winter`로 옮겨졌지만, 이는 결합 이동([coupled_drive](coupled_drive.md))
이 새로 생기면서 제설(겨울) 구간에서 이 노드가 맡는 역할(결합 전
접근까지 추종)을 기준으로 묶은 코드 정리이지, **동작 범위가 겨울로
좁아진 게 아니다** — 위 설명대로 여전히 모든 계절에서 쓰인다. entry
point 이름(`follow_leader`)과 launch 배선은 그대로다.

## 실행

`mars_launch/robot.launch.py`에서 항상 뜬다(계절 무관).

## 설정

`src/mission/config/follow.yaml` — `target_distance_mm`(기본 600),
`k_linear`/`k_angular`, `distance_tolerance_mm`, `lateral_tolerance_m`,
`k_yaw`/`yaw_tolerance_deg`(2026-08-29 추가, 아래 "동작 요약" 참고),
`max_linear_mps`/`max_angular_rps`, `sensor_timeout_s`(기본 0.5),
`allow_marker_distance_fallback`(UWB 없을 때 마커 solvePnP 거리 사용 허용),
`marker_distance_ema_alpha`(마커 거리 폴백에만 적용되는 z 잡음 완화용
지수이동평균 계수).

**`k_yaw`는 2026-09-03부터 `follow.yaml`에서 0.2로 켜져 있다**
(1호기가 회전하면 2호기도 정면을 맞춰 따라 돌아야 한다 — TODO 26번).
yaw 20도(0.35rad)에서 0.07 rad/s가 나오는 크기라 좌우항(`k_angular`
1.0)과 서로 밀어내지 않는다. **부호는 아직 미검증이다** — 코드가
`+k_yaw*yaw`인데 규약을 실물로 확인한 적이 없어, 반대면 추종이 더
나빠진다. 1호기를 좌우로 돌려 보며 부호부터 가려야 한다.
무한궤도는 조향 입력이 하나라 좌우항과 yaw항이 싸울 수 있다(결합
정렬에서 그래서 yaw를 뺐다) — 진동이 생기면 `k_yaw`부터 줄인다.

노드 자체의 선언 기본값은 여전히 0.0(비활성)이다 — 부호/크기를 아직 실측하지 않았다.
0.0으로 두면 이전(yaw 항 도입 전)과 완전히 같게 동작한다(회귀 없음,
`test_default_k_yaw_is_zero_no_behaviour_change`로 검증). 실측 순서:
로봇을 세워두고 마커(리더)를 손으로 천천히 돌려보며 `follow` 모드에서
`mission/follow/status`의 `yaw_deg`/`angular`을 관찰 → 작은 값(예 0.01)
부터 올려 로봇이 마커 정면을 향해 도는 방향인지 확인 → 반대 방향이면
부호를 뒤집는다.

## 토픽

- 구독: `mission/action` (String JSON, arbiter가 재발행), `marker/pose`
  (PoseStamped), `uwb/distance_mm` (Float32)
- 발행: `cmd_vel/follow` (Twist, arbiter의 SOURCES 중 하나), `mission/follow/status`
  (String JSON — phase/active/marker_ready/uwb_ready/reason/linear/angular/
  `yaw_deg`(2026-08-29 추가, 마커가 fresh할 때만 값이 들어가고 아니면
  `null`))

## 동작 요약

- `action != 'follow'`면 항상 정지 명령(Twist())을 낸다 — 인식/계산 자체는
  계속 돌지만 출력이 0.
- **센서 우선순위**: UWB가 `sensor_timeout_s` 안에 있으면 UWB 거리 사용.
  없으면 `allow_marker_distance_fallback`이 켜져 있을 때만 마커 solvePnP
  거리(`pose.position.z * 1000`)로 대체 — 꺼져 있으면 `uwb_timeout`으로
  정지.
- 마커 자체가 `sensor_timeout_s` 안에 없으면 UWB 유무와 무관하게 정지
  (`marker_timeout`).
- 선속도는 거리 오차 비례(P). 각속도(조향)는 **좌우 오프셋(`k_angular`,
  `pose.position.x`) + yaw 정면 정렬(`k_yaw`, `pose.orientation`에서
  뽑은 로봇의 실제 좌우 회전) 두 항을 더한 값**이다(`_combine_angular`)
  — 둘 다 단순 P 제어이고 PID가 아니다(결합의 `coupling.py`와 다름).
  `_heading_from_quaternion`으로 쿼터니언에서 직접 뽑는다 —
  [marker_vision](../base/marker_vision.md)이 roll/pitch/yaw를
  쿼터니언으로 인코드한 것의 역변환이라 `marker/pose` 하나만 구독해도
  충분하고, `marker/status`를 따로 구독할 필요가 없다. **주의(2026-08-30
  실측으로 확인)**: 여기서 뽑는 건 쿼터니언의 "yaw" 성분이 아니라
  **"pitch" 성분**이다 — `aruco_tracker.py`의 roll/pitch/yaw는 마커
  자신의 로컬 축 기준일 뿐이라 마커 마운트 방향을 모르는데, 이 로봇은
  마커판이 뒤쪽에 수직으로 붙어있어서 로봇의 실제 좌우 회전(월드 yaw)이
  마커 로컬 **Y축(pitch_deg)** 회전으로 나타나기 때문이다(로컬 Z축인
  `yaw_deg`는 오히려 로봇의 롤/뱅킹에 해당 — 자세한 실측 경위는
  [TODO.md](../../TODO.md) 11번 참고). `angular.z`가 하나의 `Twist`로
  나가면 `motor.py`가 `linear.x`(공통 전진력)와 `angular.z`(좌우
  차동)로 두 바퀴 파워를 나누므로, 이 노드 안에서 좌/우 파워를 직접
  계산할 필요가 없다.

## 연결

- 위: `drive/arbiter`(`mission/action`), `base/marker_vision`(`marker/pose`),
  `base/uwb`(`uwb/distance_mm`).
- 아래: `drive/arbiter`가 `cmd_vel/follow`를 후보로 받아 `mode=follow`이고
  `_source()`가 `follow`일 때만 채택한다.

## 구현 상태

완성(거리·좌우 오프셋 제어). `follow.yaml`에
`allow_marker_distance_fallback: true`가 설정돼 있어, UWB
미장착 상태([uwb.md](../base/uwb.md))에서도 마커 거리만으로 추종이
동작한다. **yaw 정면 정렬 항은 코드는 완성이지만(2026-08-29) `k_yaw`
기본값이 0.0(비활성)이라 실제 조향에는 아직 영향을 주지 않는다** —
실측 튜닝 전까지는 기존과 동일하게 거리+좌우 오프셋만으로 동작한다.
`k_yaw`가 0.0이라 `master`에 있어도 안전(자동화 테스트로 회귀 없음
확인됨)해서 2026-08-30에 `master`로 가져왔다 — 실물로 부호/크기를
확인해 `k_yaw`를 0이 아닌 값으로 켜기 전까지는 이 상태를 유지한다
([TODO.md](../../TODO.md) 11번). **같은 날(2026-08-30) 축 오인식
버그를 실측으로 발견·수정**: `_yaw_from_quaternion`(개명 후
`_heading_from_quaternion`)이 쿼터니언의 yaw(Z축) 성분을 뽑고
있었는데, 이 로봇의 마커 마운트 기준으로는 그게 실제로는 로봇의
롤(뱅킹)에 해당하고, 로봇의 진짜 좌우 회전(월드 yaw)은 pitch(Y축)
성분으로 나온다는 걸 확인해 그쪽을 뽑도록 고쳤다 — `k_yaw`는 여전히
0.0이라 이 수정 자체도 실제 조향에는 영향 없음.

## 알려진 이슈

없음. **2026-08-28**: [marker_vision](../base/marker_vision.md)의 ArUco
pose 캘리브레이션이 확정돼 `marker/pose`(PoseStamped)가 다시 나온다.
재검토 결과 이 노드는 애초에 `PoseStamped`의 `position.x`/`.z`를 타입
그대로 구독하므로(JSON 필드명을 직접 찾지 않음) 필드명 불일치 문제가
없었다 — pose가 켜지자마자 폴백이 정상 동작한다. 다만 solvePnP `z`
(거리)는 멀수록 잡음이 커져서(60cm서 약 7%) `marker_distance_ema_alpha`로
지수이동평균해 마커 폴백일 때만 속도 명령 떨림을 줄이도록 추가했다
(UWB 사용 시에는 영향 없음). `src/mission/test/
test_follow_leader_marker_pose.py`로 스무딩 동작을 자동화 테스트했다.

**2026-08-29**: yaw 정면 정렬 항(`k_yaw`) 추가. 쿼터니언→yaw 역변환은
`marker_vision.py`의 순변환과 왕복 검증(자동화 테스트)까지 마쳤지만,
**실제 로봇이 yaw 오차에 어느 방향으로 반응해야 하는지(부호)는 아직
실측하지 않았다** — 기본값 0.0으로 안전하게 꺼둔 이유다. 실측 전에는
`master`로 merge하지 않는다.
