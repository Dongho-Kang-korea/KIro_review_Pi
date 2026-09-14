# camera — `src/base/base/camera.py`

**최신화**: 2026-09-03

## 역할

Picamera2로 촬영한 영상을 ROS 토픽(`CompressedImage`)과 별도 MJPEG HTTP
스트림으로 동시에 발행한다. 카메라가 없거나 `picamera2` import가 실패하면
더미 프레임("Dummy Frame #N")을 대신 발행해 통신 시험을 계속할 수 있다.

## 실행

`mars_launch/robot.launch.py`, `manual.launch.py`(`console:=true`일 때만),
`test_drive.launch.py`에서 뜬다. `PYTHONPATH=/usr/local/lib/python3/dist-packages`
를 launch가 주입한다 — `picamera2`가 시스템 site-packages에만 있기 때문.

## 설정

`src/base/config/camera.yaml` — 해상도(`width`/`height`), `frame_rate`,
`jpeg_quality`, `stream_port`(기본 8000), `rotation`(0/90/180/270),
`full_fov`(센서 전체 영역 사용 여부). 숫자는 이 파일이 최신 출처다.

## 토픽

- 발행: `camera/image/compressed` (CompressedImage, ROS 항상 발행)
- 별도: `http://<IP>:8000/stream.mjpg` — MJPEG, 구독자가 있을 때만 프레임 갱신

## 동작 요약

- `full_fov: true`면 센서 `PixelArraySize` 전체로 `ScalerCrop`을 설정해
  확대 체감을 줄인다. 단 이건 디지털 줌만 없애는 것이고, 출력
  `width`/`height`(4:3 등)가 센서 원본 비율(imx708은 16:9)과 다르면
  Picamera2가 `preserve_ar`로 좌우를 추가로 crop한다 — 잘림 없이 진짜
  전체 화각을 보려면 출력도 16:9(예: 640x360)로 맞춰야 한다.
  2026-08-28에 기본값을 640x480(4:3, 좌우 약 25% crop)에서 640x360(16:9,
  crop 없음)으로 바꿨고, 같은 날 ArUco pose 정확도(멀리 있는 마커가
  화면에서 너무 작게 잡히는 문제) 때문에 다시 1280x720(16:9, crop
  없음)으로 올렸다 — marker.yaml 재캘리브레이션과 세트다.
- `rotation`은 180일 때 `cv2.flip(-1)`, 90/270일 때 `cv2.rotate`로 처리.
- ROS 발행과 MJPEG 스트리밍은 분리돼 있다 — MJPEG는 `stream_output.has_subscribers`
  일 때만 프레임을 갱신해 불필요한 인코딩을 줄인다.
- 역방향 DNS 조회를 생략(`server_bind` 오버라이드)해 HTTP 서버 기동 지연을 막는다.
- **2026-09-01**: 영상 노드(카메라·marker_vision·cargo_load)가 여러 개
  동시에 돌아 Pi 5 4코어에 부하가 몰려(실측 load 15) 마커 검출률이
  0%까지 떨어지는 걸 확인, `frame_rate`를 30→15로 낮췄다. 같은 이유로
  `cv2.setNumThreads(1)`을 모듈 임포트 시점에 호출한다 — 영상 노드가
  프로세스 단위로 이미 병렬이라 OpenCV 내부 스레드까지 병렬화하면
  오히려 코어를 두고 경쟁해 총 처리량이 준다. `jpeg_quality`도 70→82로
  올렸다 — marker_vision이 이 JPEG를 직접 디코딩해 검출하므로 경계
  압축 손실을 줄이는 쪽이 인식률에 유리하다.

## 연결

- 위: 없음(하드웨어 최상단 입력).
- 아래: `base/marker_vision`(ArUco 마커 인식),
  `mission/cargo_load`(적색 검출)가 모두 `camera/image/compressed`를 구독한다.
  `drive/test_console`과 `ui/console`도 같은 토픽을 구독해 화면에 띄운다.

## 구현 상태

완성. 실물 카메라 유무와 무관하게 항상 기동(더미 프레임 폴백).

## 알려진 이슈

없음 — 발견되면 [TODO.md](../../TODO.md)에 등록한다.
