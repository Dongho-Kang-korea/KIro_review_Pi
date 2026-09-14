# calibrate_camera — `src/base/base/calibrate_camera.py`

**최신화**: 2026-08-28 (ArUco 단일 마커 모드 추가)

## 역할

체커보드로 카메라 내부 파라미터(`camera_matrix`, `dist_coeffs`)를 구하는
**일회성 커미셔닝 도구**다. `robot.launch.py` 등 실제 운용 launch에는
넣지 않는다 — 사람이 필요할 때 직접 `ros2 run`으로 띄운다.

`base/camera`가 카메라 장치(Picamera2)를 이미 물고 있으므로, 이 노드는
장치를 직접 열지 않고 `camera/image/compressed` 토픽을 구독해서 쓴다.
그래서 **`base/camera`를 먼저 띄운 상태에서 이 노드를 같이 돌려야 한다.**

## 실행

어떤 launch에도 포함되지 않는다. 수동으로:

```bash
ros2 run base camera --ros-args -r __ns:=/unit2 \
  --params-file install/base/share/base/config/camera.yaml
ros2 run base calibrate_camera --ros-args -r __ns:=/unit2
```

`http://<IP>:8002/stream.mjpg`를 열어 체커보드를 여러 각도·거리로
비춰준다. 코너가 잡히면 초록 점이 그려지고 자동으로 한 장씩 캡처된다
(`min_capture_interval_s` 간격). 지정한 장수(`target_samples`)가 모이면
자동으로 계산하고 파일로 저장한다.

**체커보드가 없으면** `use_aruco_target:=true`로 갖고 있는 ArUco 마커
하나로 대신할 수 있다(2026-08-28 추가, 프린터 없는 상황에서 실제로 이걸로
캘리브레이션을 진행했다):

```bash
ros2 run base calibrate_camera --ros-args -r __ns:=/unit2 \
  -p use_aruco_target:=true -p target_samples:=40
```

## 설정 (런치 파일 없음 — `ros2 run` 인자로 넘긴다)

체커보드 모드: `board_cols`/`board_rows`(내부 코너 개수, 기본 9x6),
`square_size_mm`(기본 20.0 — **`camera_matrix`/`dist_coeffs`의 정확도에는
영향 없다**, 체커보드 객체점 전체를 스케일해도 초점거리·주점은 수학적으로
바뀌지 않기 때문. 정확히 안 재도 된다).

ArUco 모드(`use_aruco_target:=true`): `aruco_dictionary`(기본
`DICT_4X4_50`), `aruco_allowed_ids_json`(기본 `[1,2,3]`),
`aruco_target_id`(기본 -1, 가장 크게 보이는 허용 마커), `aruco_marker_size_mm`
(기본 40.0, `marker.yaml`의 `marker_size_mm`와 동일), `zone_grid`(화면을
NxN 구역으로 나눔, 기본 3), `min_zone_coverage`(최소 이 구역 수 이상
덮여야 완료, 기본 6/9), `max_samples_per_zone`(한 구역에서 이 이상은 안
받음, 기본 6 — 한 자리에서만 계속 찍는 걸 막는다).

공통: `target_samples`(체커보드 기본 20, ArUco 기본 40 — 마커 하나는
장당 점이 4개뿐이라 체커보드보다 훨씬 많이 필요하다),
`min_capture_interval_s`(기본 1.0), `preview_port`(기본 8002),
`output_path`(기본 `src/base/config/camera_calib.yaml`).

## 토픽

- 구독: `camera/image/compressed`, `calibration/command` (String —
  `"reset"`을 보내면 모은 샘플을 지우고 처음부터 다시 시작)
- 발행: `calibration/status` (String JSON — `mode`/`samples_collected`/
  `target_samples`/`zones_covered`/`zones_total`/`zone_counts`/`done`/
  `result`), 별도 MJPEG 미리보기(`http://<IP>:8002/stream.mjpg`, ROS 토픽
  아님 — `base/camera`와 같은 패턴). ArUco 모드에서는 미리보기에 3x3 구역
  격자와 구역별 캡처 수도 같이 그려진다.

## 동작 요약

- **체커보드 모드**: 매 프레임 `cv2.findChessboardCorners` → 찾으면
  `cornerSubPix`로 정밀화 → `min_capture_interval_s` 이상 지났으면 샘플로
  추가.
- **ArUco 모드**: `base.aruco_tracker.ArucoMarkerTracker`를 그대로 재사용해
  검출 → 검출된 마커 중심이 속한 구역을 계산 → 그 구역이 아직
  `max_samples_per_zone` 미만이고 시간 간격도 지났으면 샘플로 추가.
  `target_samples`장 **그리고** `min_zone_coverage`개 구역이 모두 채워져야
  계산을 시작한다(장수만 채우고 한 자리에서 안 움직이면 끝나지 않는다).
  둘 다 못 채운 채 `target_samples`의 2배(`hard_cap`)에 도달하면 그냥
  있는 것으로 계산한다. 장당 점이 4개뿐이라 과적합을 줄이려고
  `cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3`로 왜곡 모델을 단순화한다.
- `target_samples`(+ ArUco면 구역 조건까지) 충족되면 `cv2.calibrateCamera`
  실행 → 평균 재투영 오차(px) 계산 → `output_path`에 결과 저장.
- 저장 형식은 `marker.yaml`의 `ros__parameters` 아래에 그대로 붙여넣을 수
  있는 4줄(`camera_matrix_json`/`dist_coeffs_json`/`calibration_width`/
  `calibration_height`) — **자동으로 `marker.yaml`을 고치지 않는다.**
  결과를 검토한 뒤 사람이 직접 옮긴다.

## 연결

- 위: `base/camera`(`camera/image/compressed`).
- 아래: 없음(결과 파일을 사람이 `marker.yaml`에 수동 반영하면
  [marker_vision](marker_vision.md)이 그걸 읽는다).

## 구현 상태

**완성, 두 모드 다 실물로 끝까지 써봤다**. ArUco 단일 마커 모드로 먼저
캘리브레이션했으나 30cm 실측 검증에서 50% 오차가 나와 폐기 — 프린터가
없어 시도한 대체 경로였지만, 점이 장당 4개뿐이라 정확도가 부족했다.
이후 노트북 화면에 체커보드를 띄우는 방식(체커보드 모드)으로
재캘리브레이션했고, 30cm·60cm 실측 검증을 통과해 `marker.yaml`에
반영했다 — 결과적으로 **체커보드 모드(화면 띄우기 포함)를 권장한다.**
자세한 시행착오는 [marker_vision 문서](marker_vision.md) 참고.

## 알려진 이슈

없음 — 발견되면 [TODO.md](../../TODO.md)에 등록한다.
