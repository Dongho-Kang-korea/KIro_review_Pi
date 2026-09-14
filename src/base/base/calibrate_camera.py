"""체커보드 또는 ArUco 마커 하나로 카메라 내부 파라미터를 구하는 도구.

일회성 커미셔닝 도구다 — robot.launch.py 등 실제 운용 launch에는 넣지
않는다. base/camera 노드가 카메라 장치를 이미 물고 있으므로, 이 노드는
Picamera2를 직접 열지 않고 `camera/image/compressed`를 구독해서 쓴다.
그래서 카메라 노드를 먼저 띄운 상태에서 이 노드를 같이 돌리면 된다.

사용법 (예: 2호기 네임스페이스)::

    ros2 run base camera --ros-args -r __ns:=/unit2 \
        --params-file install/base/share/base/config/camera.yaml
    ros2 run base calibrate_camera --ros-args -r __ns:=/unit2

기본은 체커보드(9x6 내부 코너) 방식이다. 체커보드가 없으면
``use_aruco_target:=true``로 갖고 있는 ArUco 마커 하나로 대신할 수 있다::

    ros2 run base calibrate_camera --ros-args -r __ns:=/unit2 \
        -p use_aruco_target:=true -p target_samples:=40

**체커보드보다 정확도가 떨어진다** — 체커보드는 한 장에 코너 54개가
나오지만 마커 하나는 4개뿐이라, 그만큼 장수를 훨씬 많이(기본 40장) 모아야
하고 화면 구석구석 골고루 비춰야 한다. 그래서 ArUco 모드는 화면을
3x3 구역으로 나눠 구역별 캡처 수를 세고, 정해진 구역 수
(`min_zone_coverage`, 기본 6/9) 이상 덮여야 계산을 마친다 — 한 자리에서만
찍으면 아무리 많이 찍어도 안 끝난다. 왜곡계수도 고차항(k3, 접선왜곡)은
빼고 계산해 적은 점 수로 인한 과적합을 줄인다.

`http://<IP>:<preview_port>/stream.mjpg`를 열어 진행 상황을 실시간으로
본다 — 코너/마커가 잡히면 자동으로 한 장씩 캡처하고, 조건이 다 차면 바로
계산해서 결과를 저장한다.

**체커보드 사각형/마커 크기 자체의 실측 정확도는 camera_matrix/dist_coeffs
결과에 영향이 없다** — 이 값들(초점거리·주점, 픽셀 단위)은 객체점을
통째로 스케일해도 수학적으로 바뀌지 않는다(스케일은 계산되는 보드의
위치/자세에만 반영된다). ArUco 모드의 `aruco_marker_size_mm` 기본값은
`marker.yaml`의 `marker_size_mm`(사용자가 확정한 40mm, 검은 패턴 기준)와
맞춰뒀다.

결과는 `output_path`(기본: 이 저장소의 `src/base/config/camera_calib.yaml`)에
`marker.yaml`로 그대로 옮겨 붙일 수 있는 형태로 저장된다 — 자동으로
`marker.yaml`을 고치지는 않는다. 계산값을 검토한 뒤 사람이 직접 옮긴다.
"""
import json
import io
import socketserver
import threading
import time
from http import server

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from base.aruco_tracker import ArucoMarkerTracker


class StreamingOutput(io.BufferedIOBase):
    """base/camera.py의 MJPEG 패턴 재사용 — 미리보기 전용."""

    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


stream_output = StreamingOutput()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != '/stream.mjpg':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()
        try:
            while True:
                with stream_output.condition:
                    stream_output.condition.wait(timeout=2.0)
                    frame = stream_output.frame
                if frame is None:
                    continue
                self.wfile.write(b'--FRAME\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(frame))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, format, *args):
        pass


class ThreadedHTTPServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


DEFAULT_OUTPUT = '/home/ubuntu/ros2_ws/src/base/config/camera_calib.yaml'


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')

        self.declare_parameter('use_aruco_target', False)
        self.use_aruco = bool(self.get_parameter('use_aruco_target').value)

        # 체커보드 설정 (use_aruco_target:=false일 때)
        self.declare_parameter('board_cols', 9)   # 내부 코너 개수(가로)
        self.declare_parameter('board_rows', 6)   # 내부 코너 개수(세로)
        self.declare_parameter('square_size_mm', 20.0)  # 정확도에 영향 없음(위 docstring)

        # ArUco 단일 마커 설정 (use_aruco_target:=true일 때)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('aruco_allowed_ids_json', '[1,2,3]')
        self.declare_parameter('aruco_target_id', -1)
        self.declare_parameter('aruco_marker_size_mm', 40.0)  # marker.yaml과 동일
        self.declare_parameter('zone_grid', 3)         # 화면을 NxN 구역으로 나눔
        self.declare_parameter('min_zone_coverage', 6)  # 최소 이만큼 구역이 덮여야 완료
        self.declare_parameter('max_samples_per_zone', 6)  # 한 구역에 이 이상은 안 받음

        # 마커 하나는 코너가 4개뿐이라 체커보드보다 훨씬 많이 모아야 한다.
        self.declare_parameter('target_samples', 40 if self.use_aruco else 20)
        self.declare_parameter('min_capture_interval_s', 1.0)
        self.declare_parameter('preview_port', 8002)
        self.declare_parameter('output_path', DEFAULT_OUTPUT)

        self.cols = int(self.get_parameter('board_cols').value)
        self.rows = int(self.get_parameter('board_rows').value)
        self.square_size = float(self.get_parameter('square_size_mm').value)
        self.target_samples = int(self.get_parameter('target_samples').value)
        self.min_interval = float(self.get_parameter('min_capture_interval_s').value)
        self.output_path = str(self.get_parameter('output_path').value)
        self.zone_grid = max(1, int(self.get_parameter('zone_grid').value))
        self.min_zone_coverage = min(
            self.zone_grid * self.zone_grid,
            int(self.get_parameter('min_zone_coverage').value))
        self.max_samples_per_zone = max(
            1, int(self.get_parameter('max_samples_per_zone').value))
        self.hard_cap = self.target_samples * 2

        if self.use_aruco:
            marker_size = float(self.get_parameter('aruco_marker_size_mm').value)
            half = marker_size / 2.0
            self.objp = np.array([
                [-half, half, 0.0], [half, half, 0.0],
                [half, -half, 0.0], [-half, -half, 0.0],
            ], dtype=np.float32)
            allowed_ids = json.loads(str(
                self.get_parameter('aruco_allowed_ids_json').value))
            self.aruco_tracker = ArucoMarkerTracker(
                dictionary_name=str(self.get_parameter('aruco_dictionary').value),
                allowed_ids=allowed_ids,
                target_id=int(self.get_parameter('aruco_target_id').value),
                marker_size_mm=marker_size)
        else:
            self.objp = np.zeros((self.rows * self.cols, 3), np.float32)
            self.objp[:, :2] = (
                np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square_size)
            self.aruco_tracker = None

        self.objpoints = []   # 3D 객체점 (샘플마다)
        self.imgpoints = []   # 화면 픽셀 2D 점 (샘플마다)
        self.zone_of_sample = []  # 샘플마다 어느 구역이었는지(디버그/카운트용)
        self.zone_counts = {}     # zone_index -> 캡처 수
        self.image_size = None
        self.last_capture = 0.0
        self.done = False
        self.result = None
        self.lock = threading.Lock()

        self.status_pub = self.create_publisher(String, 'calibration/status', 10)
        self.create_subscription(
            CompressedImage, 'camera/image/compressed', self._on_image, 10)
        self.create_subscription(String, 'calibration/command', self._on_command, 10)
        self.create_timer(0.5, self._publish_status)
        if self.use_aruco:
            self.get_logger().info(
                f'ArUco target mode | marker={marker_size}mm zones='
                f'{self.zone_grid}x{self.zone_grid} target={self.target_samples} '
                f'samples, {self.min_zone_coverage}+ zones -> {self.output_path}')
        else:
            self.get_logger().info(
                f'Checkerboard {self.cols}x{self.rows} internal corners, '
                f'target={self.target_samples} samples -> {self.output_path}')

    def _on_command(self, msg):
        if str(msg.data).strip().lower() == 'reset':
            with self.lock:
                self.objpoints.clear()
                self.imgpoints.clear()
                self.zone_of_sample.clear()
                self.zone_counts.clear()
                self.done = False
                self.result = None
                self.last_capture = 0.0
            self.get_logger().info('Calibration samples reset')

    def _zone_index(self, cx, cy, width, height):
        gx = min(self.zone_grid - 1, max(0, int(cx / max(1, width) * self.zone_grid)))
        gy = min(self.zone_grid - 1, max(0, int(cy / max(1, height) * self.zone_grid)))
        return gy * self.zone_grid + gx

    def _draw_zone_grid(self, frame, width, height):
        for i in range(1, self.zone_grid):
            x = width * i // self.zone_grid
            y = height * i // self.zone_grid
            cv2.line(frame, (x, 30), (x, height), (90, 90, 90), 1)
            cv2.line(frame, (0, y), (width, y), (90, 90, 90), 1)
        for zone in range(self.zone_grid * self.zone_grid):
            gy, gx = divmod(zone, self.zone_grid)
            cx = width * gx // self.zone_grid + 6
            cy = max(48, height * gy // self.zone_grid + 20)
            count = self.zone_counts.get(zone, 0)
            colour = (40, 220, 40) if count > 0 else (110, 110, 220)
            cv2.putText(frame, str(count), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, colour, 2)

    def _detect_checkerboard(self, frame, gray):
        found, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return None
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(frame, (self.cols, self.rows), corners, found)
        centre = corners.reshape(-1, 2).mean(axis=0)
        return corners, centre

    def _detect_aruco(self, frame):
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return None
        try:
            result = self.aruco_tracker.process(jpeg.tobytes())
        except ValueError:
            return None
        annotated = result.pop('_jpeg', None)
        if annotated is not None:
            decoded = cv2.imdecode(
                np.frombuffer(annotated, dtype=np.uint8), cv2.IMREAD_COLOR)
            if decoded is not None:
                frame[:, :, :] = decoded
        if not result.get('detected') or 'corners_px' not in result:
            return None
        corners = np.asarray(result['corners_px'], dtype=np.float32).reshape(4, 1, 2)
        centre = corners.reshape(-1, 2).mean(axis=0)
        return corners, centre

    def _on_image(self, msg):
        frame = cv2.imdecode(
            np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        height, width = frame.shape[:2]
        self.image_size = (width, height)

        with self.lock:
            done = self.done
            count = len(self.objpoints)
            zones_covered = sum(1 for c in self.zone_counts.values() if c > 0)

        if not done:
            if self.use_aruco:
                detection = self._detect_aruco(frame)
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detection = self._detect_checkerboard(frame, gray)

            if detection is not None:
                corners, centre = detection
                zone = self._zone_index(centre[0], centre[1], width, height)
                now = time.monotonic()
                zone_full = self.zone_counts.get(zone, 0) >= self.max_samples_per_zone
                if (now - self.last_capture >= self.min_interval and
                        count < self.hard_cap and not zone_full):
                    with self.lock:
                        self.objpoints.append(self.objp.copy())
                        self.imgpoints.append(corners)
                        self.zone_of_sample.append(zone)
                        self.zone_counts[zone] = self.zone_counts.get(zone, 0) + 1
                        count = len(self.objpoints)
                        zones_covered = sum(
                            1 for c in self.zone_counts.values() if c > 0)
                    self.last_capture = now
                    self.get_logger().info(
                        f'Captured {count}/{self.target_samples} '
                        f'(zones {zones_covered}/{self.zone_grid**2})')
                    ready = count >= self.target_samples and (
                        not self.use_aruco or
                        zones_covered >= self.min_zone_coverage)
                    if ready or count >= self.hard_cap:
                        self._calibrate()

        if self.use_aruco:
            self._draw_zone_grid(frame, width, height)
            label = (f'DONE err={self.result["reprojection_error_px"]:.3f}px'
                     if done and self.result else
                     f'{count}/{self.target_samples} | zones '
                     f'{zones_covered}/{self.zone_grid**2} (need {self.min_zone_coverage})')
        else:
            label = (f'DONE err={self.result["reprojection_error_px"]:.3f}px'
                     if done and self.result else f'{count}/{self.target_samples} captured')
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(frame, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2)
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            stream_output.write(jpeg.tobytes())

    def _calibrate(self):
        with self.lock:
            objpoints = list(self.objpoints)
            imgpoints = list(self.imgpoints)
        self.get_logger().info(
            f'{len(objpoints)}장으로 캘리브레이션 계산 중...')
        # ArUco 모드는 장당 점이 4개뿐이라 고차 왜곡항까지 풀면 과적합되기
        # 쉽다 — 접선왜곡과 k3을 빼서 모델을 단순하게 유지한다.
        flags = (cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3
                 if self.use_aruco else 0)
        ok, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, self.image_size, None, None, flags=flags)
        if not ok:
            self.get_logger().error('cv2.calibrateCamera 실패 — 다시 시도하세요'
                                    ' (calibration/command에 reset 발행)')
            return

        errors = []
        for objp, imgp, rvec, tvec in zip(objpoints, imgpoints, rvecs, tvecs):
            projected, _ = cv2.projectPoints(
                objp, rvec, tvec, camera_matrix, dist_coeffs)
            error = cv2.norm(imgp, projected, cv2.NORM_L2) / len(projected)
            errors.append(float(error))
        mean_error = float(np.mean(errors))

        self.result = {
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.reshape(-1).tolist(),
            'width': self.image_size[0],
            'height': self.image_size[1],
            'samples': len(objpoints),
            'reprojection_error_px': mean_error,
        }
        with self.lock:
            self.done = True
        self.get_logger().info(
            f'캘리브레이션 완료: 평균 재투영 오차 {mean_error:.3f}px '
            f'({len(objpoints)}장) -> {self.output_path}')
        self._write_output()

    def _write_output(self):
        r = self.result
        camera_matrix_json = json.dumps(r['camera_matrix'], separators=(',', ':'))
        dist_coeffs_json = json.dumps(r['dist_coeffs'], separators=(',', ':'))
        source = (f'ArUco 마커 1개({self.zone_grid}x{self.zone_grid} 구역 커버리지 방식)'
                 if self.use_aruco else f'체커보드 {self.cols}x{self.rows} 내부 코너')
        text = (
            f"# camera_calibrate 결과 — {time.strftime('%Y-%m-%dT%H:%M:%S')}\n"
            f"# 대상: {source}\n"
            f"# 샘플 {r['samples']}장, 평균 재투영 오차 {r['reprojection_error_px']:.3f}px\n"
            f"# 아래 4줄을 src/base/config/marker.yaml의 ros__parameters 아래에\n"
            f"# 그대로 붙여넣으세요 (자동 반영 아님 — 검토 후 직접 옮길 것).\n"
            f"    camera_matrix_json: '{camera_matrix_json}'\n"
            f"    dist_coeffs_json: '{dist_coeffs_json}'\n"
            f"    calibration_width: {r['width']}\n"
            f"    calibration_height: {r['height']}\n"
        )
        try:
            with open(self.output_path, 'w') as f:
                f.write(text)
        except OSError as exc:
            self.get_logger().error(f'{self.output_path} 저장 실패: {exc}')

    def _publish_status(self):
        with self.lock:
            count = len(self.objpoints)
            done = self.done
            result = dict(self.result) if self.result else None
            zones_covered = sum(1 for c in self.zone_counts.values() if c > 0)
            zone_counts = dict(self.zone_counts)
        payload = {
            'mode': 'aruco' if self.use_aruco else 'checkerboard',
            'board_cols': self.cols, 'board_rows': self.rows,
            'samples_collected': count, 'target_samples': self.target_samples,
            'zones_covered': zones_covered,
            'zones_total': self.zone_grid ** 2,
            'min_zone_coverage': self.min_zone_coverage,
            'zone_counts': zone_counts,
            'done': done, 'output_path': self.output_path, 'result': result,
        }
        self.status_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    port = int(node.get_parameter('preview_port').value)
    httpd = ThreadedHTTPServer(('0.0.0.0', port), StreamingHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    node.get_logger().info(f'Preview: http://<IP>:{port}/stream.mjpg')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
