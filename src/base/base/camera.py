#!/usr/bin/env python3
"""카메라 영상을 ROS 토픽과 MJPEG로 발행한다.

launch가 libcamera 경로를 주입하며 카메라가 없으면 더미 영상을 발행한다.
full_fov는 센서 전체 영역을 사용해 확대 체감을 줄인다.
"""

import io
import logging
import socketserver
import threading
from http import server
from urllib import parse

import cv2
# 영상 노드가 여러 개 동시에 돌아 4코어에 스레드가 몰린다. 프로세스
# 단위로 이미 병렬이므로 OpenCV 내부 스레드는 1로 묶는 편이 총
# 처리량이 낫다(2026-09-01 부하 실측).
cv2.setNumThreads(1)
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
STREAM_PORT = 8000
JPEG_QUALITY = 70
IMAGE_TOPIC = 'camera/image/compressed'

# 해상도와 전체 화각은 base/config/camera.yaml에서 조정한다.
# 카메라가 없으면 통신 시험용 더미 영상을 대신 발행한다.
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


class StreamingOutput(io.BufferedIOBase):
    def __init__(self, name=""):
        self.name = name
        self.frame = None
        self.condition = threading.Condition()
        self._subscribers = 0
        self._sub_lock = threading.Lock()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

    def subscribe(self):
        with self._sub_lock:
            self._subscribers += 1

    def unsubscribe(self):
        with self._sub_lock:
            self._subscribers = max(0, self._subscribers - 1)

    @property
    def has_subscribers(self):
        with self._sub_lock:
            return self._subscribers > 0

    def close_all(self):
        with self.condition:
            self.condition.notify_all()


stream_output = StreamingOutput("camera")


class StreamingHandler(server.BaseHTTPRequestHandler):
    timeout = 10

    def do_GET(self):
        parsed_path = parse.urlparse(self.path)
        if parsed_path.path == '/stream.mjpg':
            self.serve_stream()
        else:
            self.send_error(404)
            self.end_headers()

    def serve_stream(self):
        stream_output.subscribe()
        try:
            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()

            while True:
                with stream_output.condition:
                    if not stream_output.condition.wait(timeout=2.0):
                        continue
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
        except Exception as e:
            logging.debug(f'Client disconnected: {e}')
        finally:
            stream_output.unsubscribe()

    def log_message(self, format, *args):
        pass


class ThreadedHTTPServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def server_bind(self):
        # 역방향 DNS 조회를 생략해 서버 시작 지연을 막는다.
        socketserver.TCPServer.server_bind(self)
        host, port = self.server_address[:2]
        self.server_name = host
        self.server_port = port


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')

        self.declare_parameter('width', FRAME_WIDTH)
        self.declare_parameter('height', FRAME_HEIGHT)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('jpeg_quality', JPEG_QUALITY)
        self.declare_parameter('stream_port', STREAM_PORT)
        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('rotation', 180)
        self.declare_parameter('full_fov', True)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.stream_port = int(self.get_parameter('stream_port').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.rotation = int(self.get_parameter('rotation').value) % 360
        self.full_fov = bool(self.get_parameter('full_fov').value)

        self.frame_count = 0
        self.encode_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            int(self.get_parameter('jpeg_quality').value),
        ]

        self.image_pub = self.create_publisher(CompressedImage, self.image_topic, 10)

        self.get_logger().info('Camera initialization starting...')
        try:
            from picamera2 import Picamera2
            self.picam2 = Picamera2()
            config = self.picam2.create_video_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                controls={"FrameRate": self.frame_rate}
            )
            self.picam2.configure(config)
            if self.full_fov:
                sensor_w, sensor_h = self.picam2.camera_properties['PixelArraySize']
                self.picam2.set_controls({'ScalerCrop': (0, 0, sensor_w, sensor_h)})
            self.picam2.start()
            actual = self.picam2.camera_configuration()
            self.get_logger().info(
                f'Camera started | sensor={self.picam2.camera_properties.get("PixelArraySize")} '
                f'output={actual.get("main")} full_fov={self.full_fov}')
        except Exception as e:
            self.get_logger().warn(f'Camera unavailable -> using dummy frames: {e}')
            self.get_logger().warn(
                'libcamera 미탑재/카메라 미연결 추정. PYTHONPATH=/usr/local/lib/'
                'python3/dist-packages 로 실행하거나 launch 파일을 사용하세요.')
            self.picam2 = None

        self.timer = self.create_timer(1.0 / max(1.0, self.frame_rate), self.timer_callback)

    def timer_callback(self):
        try:
            if self.picam2 is not None:
                frame = self.picam2.capture_array("main")
            else:
                frame = self.create_dummy_frame()

            if self.rotation == 180:
                frame = cv2.flip(frame, -1)
            elif self.rotation == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotation == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            ok, jpeg = cv2.imencode('.jpg', frame, self.encode_param)
            if not ok:
                return
            jpeg_bytes = jpeg.tobytes()

            # ROS는 항상 발행하고 MJPEG는 접속자가 있을 때만 갱신한다.
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            msg.data = jpeg_bytes
            self.image_pub.publish(msg)

            if stream_output.has_subscribers:
                stream_output.write(jpeg_bytes)

        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')

    def create_dummy_frame(self):
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 50
        cv2.putText(frame, f"Dummy Frame #{self.frame_count}",
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        self.frame_count += 1
        return frame

    def shutdown(self):
        if rclpy.ok():
            self.get_logger().info('Shutting down camera...')
        if self.picam2 is not None:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception:
                pass
        stream_output.close_all()
        if rclpy.ok():
            self.get_logger().info('Camera shutdown complete')


def main():
    rclpy.init()
    camera_node = CameraNode()

    http_server = ThreadedHTTPServer(('0.0.0.0', camera_node.stream_port), StreamingHandler)
    http_thread = threading.Thread(target=http_server.serve_forever, daemon=True)
    http_thread.start()

    camera_node.get_logger().info(
        f'Topic: {camera_node.image_topic} | '
        f'MJPEG: http://<IP>:{camera_node.stream_port}/stream.mjpg')

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.shutdown()
        http_server.shutdown()
        http_server.server_close()
        camera_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
