"""카메라와 상태를 확인하고 비상 정지만 제공하는 시험 화면."""
import json
import threading
import time

import rclpy
from flask import Flask, Response, jsonify, render_template
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


app = Flask(__name__, template_folder='templates')
node_ref = None

# 시험 화면은 주행 명령 없이 상태 확인과 비상 정지만 제공한다.

class FrameBuffer:
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def set(self, frame):
        with self.condition:
            self.frame = frame
            self.condition.notify_all()

    def wait(self, timeout=2.0):
        with self.condition:
            self.condition.wait(timeout)
            return self.frame


class TestConsole(Node):
    def __init__(self):
        super().__init__('test_console')
        # 규약 상태 토픽은 호기 번호를 탄다 (현재 2호기 하나).
        self.declare_parameter('unit', 2)
        self.unit = int(self.get_parameter('unit').value)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5000)
        self.frame = FrameBuffer()
        self.lock = threading.Lock()
        self.state = {
            'drive': {}, 'fleet': {}, 'motor': {}, 'marker': {}, 'cargo': {},
            'coupling': {}, 'imu': {}, 'uwb': {},
            'solenoid_locked': False,
            'camera_last': None,
        }
        self.estop_pub = self.create_publisher(Bool, 'safety/emergency_stop', 10)
        self.coupling_pub = self.create_publisher(String, 'coupling/cmd', 10)
        self.create_subscription(
            CompressedImage, 'camera/image/compressed', self._on_camera, 10)
        for topic, key in (
            ('drive/status', 'drive'), (f'/fleet/status/unit{self.unit}', 'fleet'),
            ('motor/status', 'motor'), ('marker/status', 'marker'),
            ('mission/cargo/status', 'cargo'), ('imu/status', 'imu'),
            ('mission/coupling/status', 'coupling'),
            ('uwb/status', 'uwb')):
            self.create_subscription(
                String, topic, lambda msg, name=key: self._on_json(name, msg), 10)
        self.create_subscription(Bool, 'solenoid_status', self._on_solenoid, 10)

    def _on_camera(self, msg):
        self.frame.set(bytes(msg.data))
        with self.lock: self.state['camera_last'] = time.time()

    def _on_json(self, key, msg):
        try: value = json.loads(msg.data)
        except Exception: return
        with self.lock: self.state[key] = value

    def _on_solenoid(self, msg):
        with self.lock: self.state['solenoid_locked'] = bool(msg.data)

    def snapshot(self):
        with self.lock: return json.loads(json.dumps(self.state))

    def emergency(self, stopped):
        msg = Bool(); msg.data = bool(stopped); self.estop_pub.publish(msg)

    def coupling(self, action):
        msg = String()
        msg.data = json.dumps({'action': action}, separators=(',', ':'))
        self.coupling_pub.publish(msg)


@app.get('/')
def index(): return render_template('test_console.html')


@app.get('/api/status')
def status(): return jsonify(node_ref.snapshot())


@app.post('/api/emergency-stop')
def emergency_stop():
    node_ref.emergency(True)
    return ('', 204)


@app.post('/api/reset-emergency')
def reset_emergency():
    node_ref.emergency(False)
    return ('', 204)


@app.post('/api/coupling/start')
def coupling_start():
    node_ref.coupling('start')
    return ('', 204)


@app.post('/api/coupling/cancel')
def coupling_cancel():
    # 진행 중인 결합만 멈춘다. 솔레노이드는 그대로 잠겨 있다.
    node_ref.coupling('cancel')
    return ('', 204)


@app.post('/api/coupling/unlock')
def coupling_unlock():
    # 솔레노이드 해제. 자동으로는 절대 불리지 않는 유일한 경로다 —
    # 잠금은 기계적으로 유지되고, 푸는 것은 사람이 누를 때만 한다
    # (2026-09-03 확정, TODO 23번).
    node_ref.coupling('unlock')
    return {'ok': True}


@app.get('/camera.mjpg')
def camera_stream():
    def frames():
        while rclpy.ok():
            frame = node_ref.frame.wait()
            if frame:
                yield (b'--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                       str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
    return Response(frames(), mimetype='multipart/x-mixed-replace; boundary=FRAME',
                    headers={'Cache-Control': 'no-store'})


def main(args=None):
    global node_ref
    rclpy.init(args=args); node_ref = TestConsole()
    host = str(node_ref.get_parameter('host').value)
    port = int(node_ref.get_parameter('port').value)
    server = threading.Thread(
        target=lambda: app.run(host=host, port=port, threaded=True, use_reloader=False),
        daemon=True)
    server.start()
    node_ref.get_logger().info(f'Unit 2 test console: http://{host}:{port}')
    try: rclpy.spin(node_ref)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node_ref.emergency(True)
        node_ref.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
