# uwb — `src/base/base/uwb.py`

**최신화**: 2026-09-03

## 역할

DWM1001 UWB 모듈의 거리 값을 시리얼로 읽어 발행한다. `enabled: false`
(기본, 미장착)일 때는 포트를 열지 않고 `ready: false`만 보고한다.

모듈은 태그(Tag) 모드로 설정돼 있어야 하고(공장 기본값은 앵커), 최소 하나의
앵커가 별도 전원으로 근처에서 켜져 있어야 거리가 잡힌다. 태그/앵커 모드
전환은 셸 명령(`nmt`=태그, `nma`=앵커)으로 하며, 이 노드는 모드를 바꾸지
않는다 — 장착 시 한 번 설정해 두면 된다.

## 실행

`mars_launch/robot.launch.py`, `test_drive.launch.py`에서 뜬다.

## 설정

`src/base/config/uwb.yaml` — `enabled`, `port`(2026-09-01부터
`/dev/serial/by-id/usb-SEGGER_J-Link_000760046619-if00` — `/dev/ttyACM0`는
재부팅 열거 순서에 따라 바뀔 수 있어 장치 고유 경로로 전환),
`baudrate`(기본 115200), `publish_rate`, `offset_mm`(거리 보정),
`shell_enter_delay_s`(기본 0.2초, 2026-09-01 추가 — 셸 진입 명령 간
간격), `data_timeout_s`(기본 3초, 2026-09-01 추가 — 이 시간 응답이
없으면 재연결), `reconnect_interval_s`(기본 2초, 2026-09-01 추가).

## 토픽

- 발행: `uwb/distance_mm` (Float32), `uwb/status` (String JSON —
  enabled/ready/distance_mm/error)

## 동작 요약

- DWM1001은 명령 없이는 스스로 데이터를 안 보낸다 — 매 tick마다 `lec\r`
  (거리 CSV 조회) 명령을 시리얼로 써서 다음 tick에 응답을 받는다.
- 라인 파싱은 JSON(`{"distance_mm": ...}`) 우선 시도 후, 실패하면 DWM1001 셸의
  `lec` 응답(`DIST,<n>,AN0,<라벨>,x,y,z,거리(m)`)에서 마지막 필드를 숫자로
  해석하는 폴백을 쓴다(`_parse`) — 이 폴백은 **미터 단위이므로 1000을 곱해
  mm로 환산**한다. 콤마가 없는 라인(셸 에코 `lec`, 빈 프롬프트 `dwm> ` 등)은
  무시한다.
- **2026-09-01 시리얼 통신 견고화**: 셸 진입 명령(`\r\r`)을 한 틱에 몰아
  보내면 셸이 인터럽트/플로딩되는 문제가 있어 `shell_enter_delay_s`
  간격을 두고 나눠 보내도록 고쳤다. USB 쓰기가 간헐적으로 장치를
  binary TLV 모드에 남겨 텍스트 응답이 끊기는 문제에 대응해
  `data_timeout_s` 동안 응답이 없으면 `reconnect_interval_s`마다 셸을
  자동으로 재진입한다 — 이제 시리얼 오픈 실패·응답 두절 모두 주기적
  재연결로 스스로 복구한다(`imu.py`와 비슷한 패턴이 됨. 예전에는 시리얼
  오픈 실패 시 재시도 없이 `error`만 남겼다).

## 연결

- 위: 없음(하드웨어 최상단 입력, 미장착 시 비활성).
- 아래: `mission/follow_leader`(추종 거리 1순위 — 없으면
  `allow_marker_distance_fallback`으로 마커 solvePnP 거리 사용),
  `mission/cargo_load`(approach 단계 거리 판정), `mission/coupling`은
  UWB를 쓰지 않고 마커 pose(방위각·크기)만 쓴다(2026-09-01 재작성 후에도
  동일), `fleet/follower`(상태 보고용 `uwb_mm`).

## 구현 상태

**장착·활성화됨** — 2026-08-29부터 `enabled: true` (포트는 2026-09-01부터
by-id 고정 경로, 위 "설정" 참고).
태그(`DW4823`)+앵커(`DWD00C`)로 `distance_mm`이 실제 거리를 반영해 살아있는
값으로 발행되는 것을 확인했고, 사용자가 정면 기준 거리를 육안으로 비교해
정확도를 확인했다. 다만 `offset_mm`은 아직 `0.0`(줄자 등 정확한 기준 거리로
잰 정밀 보정은 아직) — [TODO.md](../../TODO.md) 1번 항목 참고. 추종은 UWB
없이도 마커 거리 폴백으로 동작하지만, 여름 화물 적재의 `approach` 단계는
UWB가 필수다([HARDWARE.md](../../HARDWARE.md) 참고).

## 알려진 이슈

- 태그의 정지 감지(stationary detection)가 켜져 있으면 가만히 있을 때
  갱신 주기가 10초까지 느려진다 — `acts`/`aurs`로 1초 고정 갱신으로
  재설정해 둠(2026-08-29). 재설정은 플래시에 저장돼 전원 재인가에도
  유지된다. 새 모듈을 태그로 추가할 때도 같은 설정이 필요하다.
- `offset_mm` 정밀 보정, 여름 화물 적재 `approach` 단계 통합 시험은
  [TODO.md](../../TODO.md) 1번 항목에서 진행 상황을 확인한다.
