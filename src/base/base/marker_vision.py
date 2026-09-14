"""Detect Robot 1/2/3 ArUco markers from the Raspberry Pi camera stream."""
import json
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from base.aruco_tracker import ArucoMarkerTracker


class MarkerVision(Node):
    def __init__(self):
        super().__init__('marker_vision')

        # camera_matrix_json/dist_coeffs_json are empty by default -> pose
        # stays off and marker geometry below is metadata only. Fill them in
        # (from base/calibrate_camera's output) to turn pose on.
        self.declare_parameter('enabled', True)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('allowed_ids_json', '[1,2,3]')
        self.declare_parameter('target_id', -1)
        self.declare_parameter('marker_size_mm', 40.0)
        self.declare_parameter('plate_size_mm', 45.0)
        self.declare_parameter('border_bits', 1)
        self.declare_parameter('min_side_px', 20.0)
        self.declare_parameter('jpeg_quality', 82)
        self.declare_parameter('detection_timeout_s', 0.6)
        # Raw 30 fps detections are classified through a sliding window with
        # hysteresis. Acquiring is deliberately easier than declaring a loss.
        self.declare_parameter('decision_window_s', 0.35)
        self.declare_parameter('acquire_min_hits', 4)
        self.declare_parameter('acquire_hit_ratio', 0.60)
        self.declare_parameter('lost_min_samples', 6)
        self.declare_parameter('lost_miss_ratio', 0.80)
        self.declare_parameter('lost_confirm_s', 0.25)
        self.declare_parameter('status_rate_hz', 20.0)
        self.declare_parameter('clahe_fallback', True)
        self.declare_parameter('equalize_fallback', True)
        self.declare_parameter('equalize_primary', False)
        # 0이면 비활성 -- vision/image/compressed(웹 표시용) 주석 프레임만
        # 이 크기로 줄여 인코드한다. 검출/pose는 항상 원본(camera.yaml의
        # width/height) 해상도에서 그대로 계산되므로 정확도에 영향 없다.
        self.declare_parameter('stream_width', 0)
        self.declare_parameter('stream_height', 0)
        # 3축 좌표계 오버레이 초기값. 기본 꺼짐 -- 실시간성이 필요할 때는
        # 끄고, 좌표값을 눈으로 확인하고 싶을 때만 marker/axes_enable로
        # 켠다(웹 콘솔 토글 버튼).
        self.declare_parameter('draw_axes', False)
        self.declare_parameter('axes_length_mm', 30.0)
        # 비어 있으면(기본값) pose 계산을 안 한다 — calibrate_camera로 얻은
        # 값을 검토 후 여기 채워 넣기 전까지는 지금과 같은 픽셀 전용 상태다.
        self.declare_parameter('camera_matrix_json', '[]')
        self.declare_parameter('dist_coeffs_json', '[]')
        self.declare_parameter('calibration_width', 0)
        self.declare_parameter('calibration_height', 0)

        allowed_ids = json.loads(str(
            self.get_parameter('allowed_ids_json').value))
        if not isinstance(allowed_ids, list):
            raise ValueError('allowed_ids_json must contain a JSON list')
        camera_matrix, dist_coeffs = self._load_calibration()
        self.tracker = ArucoMarkerTracker(
            dictionary_name=str(self.get_parameter('dictionary').value),
            allowed_ids=allowed_ids,
            target_id=int(self.get_parameter('target_id').value),
            marker_size_mm=float(self.get_parameter('marker_size_mm').value),
            plate_size_mm=float(self.get_parameter('plate_size_mm').value),
            border_bits=int(self.get_parameter('border_bits').value),
            min_side_px=float(self.get_parameter('min_side_px').value),
            jpeg_quality=int(self.get_parameter('jpeg_quality').value),
            camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
            calibration_width=int(self.get_parameter('calibration_width').value),
            calibration_height=int(self.get_parameter('calibration_height').value),
            stream_width=int(self.get_parameter('stream_width').value),
            stream_height=int(self.get_parameter('stream_height').value),
            draw_axes=bool(self.get_parameter('draw_axes').value),
            axes_length_mm=float(self.get_parameter('axes_length_mm').value),
            clahe_fallback=bool(
                self.get_parameter('clahe_fallback').value),
            equalize_fallback=bool(
                self.get_parameter('equalize_fallback').value),
            equalize_primary=bool(
                self.get_parameter('equalize_primary').value))

        self.enabled = bool(self.get_parameter('enabled').value)
        self.lock = threading.Lock()
        self.last_result = self._empty_result()
        self.last_detection_result = None
        self.last_frame_time = None
        self.last_detection_time = None
        self.detection_history = deque()
        self.filtered_detected = False
        self.window_samples = 0
        self.window_hits = 0
        self.window_hit_ratio = 0.0
        # 상태 메시지는 5Hz로 반복되므로 그것만 보고는 실제 카메라 프레임이
        # 새로 들어왔는지 알 수 없다. 성공적으로 처리한 프레임마다 증가시켜
        # 결합 검증이 멈춘 영상을 '마커 미검출'로 오해하지 않게 한다.
        self.frame_seq = 0
        self.error = ''

        self.status_pub = self.create_publisher(String, 'marker/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'marker/pose', 10)
        self.vision_pub = self.create_publisher(
            CompressedImage, 'vision/image/compressed', 5)
        self.create_subscription(
            CompressedImage, 'camera/image/compressed', self._on_image, 10)
        self.create_subscription(Bool, 'marker/enable', self._on_enable, 10)
        self.create_subscription(String, 'marker/command', self._on_command, 10)
        self.create_subscription(
            Bool, 'marker/axes_enable', self._on_axes_enable, 10)
        status_rate = max(
            1.0, float(self.get_parameter('status_rate_hz').value))
        self.create_timer(1.0 / status_rate, self._publish_status)
        pose_note = ('pose enabled' if self.tracker.pose_available
                     else '2D pixel output only (no calibration configured)')
        self.get_logger().info(
            f'ArUco ready | dictionary={self.tracker.dictionary_name} '
            f'allowed_ids={list(self.tracker.allowed_ids)} '
            f'target_id={self.tracker.target_id} | {pose_note}')

    def _load_calibration(self):
        """camera_matrix_json/dist_coeffs_json이 비어 있으면 (None, None)."""
        try:
            camera_matrix = json.loads(str(
                self.get_parameter('camera_matrix_json').value))
            dist_coeffs = json.loads(str(
                self.get_parameter('dist_coeffs_json').value))
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid camera calibration JSON: {exc}')
            return None, None
        if not camera_matrix or not dist_coeffs:
            return None, None
        try:
            camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
            dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
            if camera_matrix.size != 9 or dist_coeffs.size < 4:
                raise ValueError('camera_matrix must be 3x3, dist_coeffs need >=4 values')
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid camera calibration values: {exc}')
            return None, None
        return camera_matrix, dist_coeffs

    @staticmethod
    def _empty_result():
        return {
            'detected': False,
            'target_detected': False,
            'detected_ids': [],
            'decoded_ids': [],
            'rejected_ids': [],
            'markers': [],
        }

    def _on_enable(self, msg):
        self.enabled = bool(msg.data)
        if not self.enabled:
            with self.lock:
                self.last_result = self._empty_result()
                self.last_detection_result = None
                self.last_detection_time = None
                self.detection_history.clear()
                self.filtered_detected = False
                self.window_samples = 0
                self.window_hits = 0
                self.window_hit_ratio = 0.0

    def _on_axes_enable(self, msg):
        self.tracker.draw_axes = bool(msg.data)
        self.get_logger().info(
            f'ArUco axes overlay -> {"on" if self.tracker.draw_axes else "off"}')

    def _on_command(self, msg):
        try:
            data = json.loads(msg.data)
            action = str(data.get('action', '')).strip().lower()
        except Exception:
            action = str(msg.data).strip().lower()
        if action == 'enable':
            self.enabled = True
        elif action == 'disable':
            self.enabled = False
            with self.lock:
                self.last_result = self._empty_result()
                self.last_detection_result = None
                self.last_detection_time = None
                self.detection_history.clear()
                self.filtered_detected = False
                self.window_samples = 0
                self.window_hits = 0
                self.window_hit_ratio = 0.0
        elif action == 'capture':
            self.get_logger().warn(
                'ArUco does not use HSV capture. Run base/calibrate_camera '
                'and set camera_matrix_json/dist_coeffs_json in marker.yaml '
                'to enable pose instead.')

    def _on_image(self, msg):
        now = time.monotonic()
        if not self.enabled:
            return
        try:
            result = self.tracker.process(bytes(msg.data))
            annotated = result.pop('_jpeg')
            self.error = ''
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(f'ArUco processing failed: {exc}')
            return

        with self.lock:
            self.last_result = result
            self.last_frame_time = now
            self.frame_seq += 1
            raw_detected = bool(result.get('detected'))
            if raw_detected:
                self.last_detection_time = now
                self.last_detection_result = dict(result)
            self._update_detection_state(now, raw_detected)

        image = CompressedImage()
        image.header = msg.header
        image.format = 'jpeg'
        image.data = annotated
        self.vision_pub.publish(image)

        if result.get('detected') and 'x_mm' in result:
            self._publish_pose(msg.header, result)

    def _update_detection_state(self, now, raw_detected):
        """Update multi-frame confidence and hysteretic detected state."""
        window = max(
            0.1, float(self.get_parameter('decision_window_s').value))
        self.detection_history.append((float(now), bool(raw_detected)))
        cutoff = float(now) - window
        while (self.detection_history and
               self.detection_history[0][0] < cutoff):
            self.detection_history.popleft()

        samples = len(self.detection_history)
        hits = sum(1 for _, detected in self.detection_history if detected)
        hit_ratio = hits / samples if samples else 0.0
        miss_ratio = 1.0 - hit_ratio
        self.window_samples = samples
        self.window_hits = hits
        self.window_hit_ratio = hit_ratio

        if not self.filtered_detected:
            if (raw_detected and
                    hits >= int(self.get_parameter('acquire_min_hits').value)
                    and hit_ratio >= float(
                        self.get_parameter('acquire_hit_ratio').value)):
                self.filtered_detected = True
        else:
            last_seen_age = (
                math.inf if self.last_detection_time is None else
                max(0.0, float(now) - self.last_detection_time))
            if (samples >= int(
                    self.get_parameter('lost_min_samples').value) and
                    miss_ratio >= float(
                        self.get_parameter('lost_miss_ratio').value) and
                    last_seen_age >= float(
                        self.get_parameter('lost_confirm_s').value)):
                self.filtered_detected = False
        return self.filtered_detected

    def _publish_pose(self, header, result):
        """선택된(target) 마커의 solvePnP pose를 PoseStamped(단위: m)로 낸다."""
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = 'camera_link'
        pose.pose.position.x = float(result['x_mm']) / 1000.0
        pose.pose.position.y = float(result['y_mm']) / 1000.0
        pose.pose.position.z = float(result['z_mm']) / 1000.0
        roll, pitch, yaw = (math.radians(float(result[k])) for k in
                           ('roll_deg', 'pitch_deg', 'yaw_deg'))
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        self.pose_pub.publish(pose)

    def _publish_status(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('detection_timeout_s').value)
        with self.lock:
            raw_result = dict(self.last_result)
            last_detection_result = (
                None if self.last_detection_result is None else
                dict(self.last_detection_result))
            frame_time = self.last_frame_time
            detection_time = self.last_detection_time
            filtered_detected = self.filtered_detected
            window_samples = self.window_samples
            window_hits = self.window_hits
            window_hit_ratio = self.window_hit_ratio
            frame_seq = self.frame_seq
        frame_fresh = frame_time is not None and now - frame_time <= timeout
        detection_age = (
            None if detection_time is None else
            max(0.0, now - detection_time))
        detection_fresh = (
            frame_fresh and filtered_detected and
            last_detection_result is not None)
        result = (
            last_detection_result if detection_fresh else raw_result)
        raw_detected = bool(raw_result.get('detected', False))
        result['detected'] = bool(detection_fresh)
        result['target_detected'] = bool(detection_fresh)
        result['raw_detected'] = raw_detected
        result['detection_age_s'] = detection_age
        result['detection_window_samples'] = window_samples
        result['detection_window_hits'] = window_hits
        result['detection_hit_ratio'] = window_hit_ratio
        result['detection_miss_ratio'] = 1.0 - window_hit_ratio
        result['lost_confirm_s'] = float(
            self.get_parameter('lost_confirm_s').value)

        msg = String()
        msg.data = json.dumps({
            'enabled': self.enabled,
            'detector_ready': True,
            'detector': 'aruco',
            'dictionary': self.tracker.dictionary_name,
            'allowed_ids': list(self.tracker.allowed_ids),
            'configured_target_id': self.tracker.target_id,
            'marker_size_mm': self.tracker.marker_size_mm,
            'plate_size_mm': self.tracker.plate_size_mm,
            'border_bits': self.tracker.border_bits,
            'pixel_only': True,
            'pose_available': bool(result.get('pose_available', False)),
            'axes_enabled': bool(self.tracker.draw_axes),
            'camera_frame_fresh': bool(frame_fresh),
            'frame_seq': int(frame_seq),
            'error': self.error,
            **result,
        }, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerVision()
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
