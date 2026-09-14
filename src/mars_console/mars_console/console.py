"""MARS 운용 콘솔 — 호기 선택 후 해당 호기 화면으로 들어간다.

로봇이 아니라 **운용 PC에서 띄운다.** ROS 2 Jazzy 가 깔린 리눅스 PC 를 로봇과
같은 망, 같은 ``ROS_DOMAIN_ID=10`` 에 두면 된다.

    export ROS_DOMAIN_ID=10
    ros2 run mars_console console            # http://localhost:5100

화면
----
``/``            호기 선택 (1·2호기, 각자 온라인 여부 표시)
``/unit/1``      1호기 — 수동 / 자동(라인트레이싱)
``/unit/2``      2호기 — 수동 / 자동 / 추종 / 조종기(RC, 웹 토글로 켬)

호기별 토픽 이름
----------------
1호기는 토픽이 루트에 있고 2호기는 ``/unit2`` 네임스페이스 아래 있다.
**두 호기가 같은 이름을 쓰면 서로의 주행 명령을 받기 때문이다.** 그래서 이
노드는 호기마다 접두사를 붙여 이름을 만든다.

모드를 거는 방법도 호기마다 다르다. 1호기는 예전 ``/mode`` 로 중재 소스를
직접 고르고(계절에 따라 comp/escort), 2호기는 ``drive/mode_cmd`` 로
manual/auto/follow 를 고른다. 그 차이를 UNITS 표가 흡수한다.

수동 주행은 **입력 감시**를 둔다. 브라우저가 ``input_timeout`` 동안 조작을
보내지 않으면 0 을 발행한다. 창을 닫거나 네트워크가 끊겼을 때 마지막 속도로
계속 가는 것을 막는다.
"""

import json
import logging
import socket
import threading
import time

import rclpy
from flask import Flask, Response, jsonify, render_template, request, stream_with_context
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

# 이 환경에서 getfqdn 역방향 조회가 서버 기동을 최대 10초 막는다 (Pi5 때와 동일).
socket.getfqdn = lambda name='': name or 'localhost'
logging.getLogger('werkzeug').setLevel(logging.WARNING)

# 1호기 계절 -> 자율주행 시 쓸 중재 소스. drive/config/drive.yaml 의
# source_topics 와 맞춰야 한다.
UNIT1_AUTO = {'spring': 'comp', 'summer': 'comp', 'autumn': 'comp',
              'winter': 'comp', 'escort': 'escort'}

UNITS = {
    1: {
        'name': '1호기',
        'prefix': '',                       # 토픽이 루트에 있다
        'modes': ['manual', 'auto'],
        'mode_topic': '/mode',              # 값이 중재 소스 이름이다
        'mode_style': 'source',
        'cmd_topic': '/cmd_vel_manual',
        'camera_topic': '/track/annotated/compressed',
        'status_topics': {
            'motor': '/motor/status', 'camera': '/camera/status',
            'section': '/mission/section', 'track': '/track/detections',
            'light': '/mission/trafficlight', 'escort': '/mission/escort/status',
            'heading': '/mission/heading_status', 'marker': '/mission/marker/progress',
        },
        'estop_topic': None,                # 1호기에는 비상 정지 토픽이 없다
        'mission_topic': None,
    },
}
# 3호기는 설계에서 빠졌다 (TODO 19, 2026-08-31 사용자 확정). 표를 만드는
# 루프는 그대로 둔다 — 호기가 다시 늘면 여기 숫자만 되돌리면 된다.
for _unit in (2,):
    UNITS[_unit] = {
        'name': f'{_unit}호기',
        'prefix': f'/unit{_unit}',
        'modes': ['manual', 'auto', 'follow', 'rc'],
        'mode_topic': f'/unit{_unit}/drive/mode_cmd',
        'mode_style': 'unit',               # manual|auto|follow 를 그대로 보낸다
        'cmd_topic': f'/unit{_unit}/cmd_vel/manual',
        # 평소엔 ArUco 박스가 그려진 vision/image/compressed를 기본으로
        # 보여주고, raw_camera_topic은 그게 끊겼을 때만 쓰는 폴백이다.
        'camera_topic': f'/unit{_unit}/vision/image/compressed',
        'raw_camera_topic': f'/unit{_unit}/camera/image/compressed',
        'status_topics': {
            'drive': f'/unit{_unit}/drive/status',
            'motor': f'/unit{_unit}/motor/status',
            'cargo': f'/unit{_unit}/mission/cargo/status',
            'coupling': f'/unit{_unit}/mission/coupling/status',
            'follow': f'/unit{_unit}/mission/follow/status',
            'marker': f'/unit{_unit}/marker/status',
            'imu': f'/unit{_unit}/imu/status',
            'uwb': f'/unit{_unit}/uwb/status',
            'fleet': f'/fleet/status/unit{_unit}',
            'rc': f'/unit{_unit}/rc/status',
        },
        'estop_topic': f'/unit{_unit}/safety/emergency_stop',
        'mission_topic': f'/unit{_unit}/mission/request',
        # 자동 결합(coupling.py)이 locking 단계에 있을 때만 이 토픽에
        # 발행하므로, 평소엔 콘솔이 직접 잠금/해제를 보내도 안 부딪힌다.
        'solenoid_topic': f'/unit{_unit}/solenoid_cmd',
        # cargo_load.py가 idle/stop일 때만 이 값을 채택한다(자동 미션
        # 시작 시 즉시 자동이 우선) — coupling.py의 solenoid_topic과
        # 같은 "평소엔 안 부딪히는 직접 발행" 패턴이다.
        'roller_manual_topic': f'/unit{_unit}/roller_manual_cmd',
        # ArUco 3축 좌표계 오버레이 on/off (marker_vision.py 구독).
        'marker_axes_topic': f'/unit{_unit}/marker/axes_enable',
        # 조종 방식(웹/RC) 토글 — fleet/rc_bridge.py 구독. 켜지면 CH10이
        # 이 호기의 drive/mode_cmd를 원격으로 정한다(2026-08-30 추가).
        'rc_enabled_topic': f'/unit{_unit}/rc/enabled',
    }

MISSION_ACTIONS = ('idle', 'approach', 'load', 'hold', 'release', 'stop')

app = Flask(__name__)
ros_node = None
shutdown = threading.Event()


class FrameBuffer:
    def __init__(self):
        self.cond = threading.Condition()
        self.frame = None

    def set(self, data):
        with self.cond:
            self.frame = data
            self.cond.notify_all()

    def wait(self, timeout=2.0):
        with self.cond:
            self.cond.wait(timeout=timeout)
            return self.frame


class Console(Node):

    def __init__(self):
        super().__init__('mars_console')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 5100)
        # 브라우저 조작이 이 시간 끊기면 수동 속도를 0 으로 만든다.
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('max_linear', 0.25)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('stale_seconds', 2.0)
        self.input_timeout = float(self.get_parameter('input_timeout').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.stale = float(self.get_parameter('stale_seconds').value)

        self.lock = threading.Lock()
        self.feeds = {unit: {} for unit in UNITS}       # key -> (payload, time)
        self.cameras = {unit: FrameBuffer() for unit in UNITS}
        self.camera_time = {unit: 0.0 for unit in UNITS}
        self.vision_time = {unit: 0.0 for unit in UNITS}
        self.camera_source = {unit: None for unit in UNITS}
        self.drive = {unit: {'linear': 0.0, 'angular': 0.0, 'time': 0.0}
                      for unit in UNITS}
        # 컨베이어(롤러) 수동 조작 — 주행과 같은 방식: 계속 재발행하다가
        # roller_hold_timeout 안에 갱신이 없으면 cargo_load.py 쪽에서
        # 자동으로 0이 된다.
        self.roller = {unit: {'value': 0.0, 'time': 0.0} for unit in UNITS}
        self.mode = {unit: 'idle' for unit in UNITS}

        self.mode_pub, self.cmd_pub, self.estop_pub, self.mission_pub = {}, {}, {}, {}
        self.solenoid_pub, self.roller_pub, self.axes_pub = {}, {}, {}
        self.rc_enabled_pub = {}
        for unit, spec in UNITS.items():
            self.mode_pub[unit] = self.create_publisher(String, spec['mode_topic'], 10)
            self.cmd_pub[unit] = self.create_publisher(Twist, spec['cmd_topic'], 10)
            if spec['estop_topic']:
                self.estop_pub[unit] = self.create_publisher(
                    Bool, spec['estop_topic'], 10)
            if spec['mission_topic']:
                self.mission_pub[unit] = self.create_publisher(
                    String, spec['mission_topic'], 10)
            if spec.get('solenoid_topic'):
                self.solenoid_pub[unit] = self.create_publisher(
                    Bool, spec['solenoid_topic'], 10)
            if spec.get('roller_manual_topic'):
                self.roller_pub[unit] = self.create_publisher(
                    Float32, spec['roller_manual_topic'], 10)
            if spec.get('marker_axes_topic'):
                self.axes_pub[unit] = self.create_publisher(
                    Bool, spec['marker_axes_topic'], 10)
            if spec.get('rc_enabled_topic'):
                self.rc_enabled_pub[unit] = self.create_publisher(
                    Bool, spec['rc_enabled_topic'], 10)
            for key, topic in spec['status_topics'].items():
                self.create_subscription(
                    String, topic,
                    lambda msg, u=unit, k=key: self._on_status(u, k, msg), 10)
            self.create_subscription(
                CompressedImage, spec['camera_topic'],
                lambda msg, u=unit: self._on_image(u, msg, 'aruco'),
                qos_profile_sensor_data)
            if spec.get('raw_camera_topic'):
                self.create_subscription(
                    CompressedImage, spec['raw_camera_topic'],
                    lambda msg, u=unit: self._on_image(u, msg, 'raw'),
                    qos_profile_sensor_data)

        self.create_timer(1.0 / 20.0, self._tick_drive)
        self.get_logger().info('mars_console ready')

    # ---------- 수신 ----------

    def _on_status(self, unit, key, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            payload = message.data      # trafficlight/section 처럼 순수 문자열도 있다
        with self.lock:
            self.feeds[unit][key] = (payload, time.monotonic())
            # 2호기 arbiter의 진짜 모드를 따라간다 — start_mode 런치 인자나
            # 다른 클라이언트의 mode_cmd로 이미 manual/auto/follow가 돼 있어도
            # 이 콘솔 자체의 mode는 set_mode()를 거치기 전엔 idle로 남아있어서,
            # 화면엔 이미 활성화된 것처럼 보이는데 _tick_drive가 계속 0만
            # 내보내는 불일치가 있었다(web_manual.launch.py의 start_mode=manual
            # 에서 발견).
            if key == 'drive' and isinstance(payload, dict):
                wire_mode = payload.get('mode')
                if wire_mode in UNITS[unit]['modes'] + ['idle']:
                    self.mode[unit] = wire_mode

    def _on_image(self, unit, message, source='raw'):
        now = time.monotonic()
        # ArUco 주석 프레임이 최근에 왔으면 더 빠른 원본 프레임이 그걸
        # 바로 덮어쓰지 않게 한다. 원본은 그게 끊겼을 때의 안전 폴백이다.
        if source == 'raw' and now - self.vision_time[unit] <= 0.75:
            return
        if source == 'aruco':
            self.vision_time[unit] = now
        self.cameras[unit].set(bytes(message.data))
        self.camera_time[unit] = now
        self.camera_source[unit] = source

    # ---------- 발행 ----------

    def _tick_drive(self):
        """수동 모드인 호기에만 20Hz 로 속도를 낸다.

        브라우저 입력이 끊기면 0 을 낸다. 그래도 계속 발행하므로 arbiter 의
        source_timeout 에는 걸리지 않고, 속도만 0 이 된다.

        RC가 전권을 쥔 호기(``rc_owns()``)에는 이 함수가 아예 아무것도
        발행하지 않는다 — API(``api_drive``/``api_roller``)에서 새 입력을
        막는 것만으론 부족하다. 이미 쥐고 있던 값(``self.drive``/
        ``self.roller``)이 있으면 API가 막힌 뒤에도 ``input_timeout``까지는
        여기서 계속 재발행돼 rc_bridge와 같은 토픽에 경쟁 상태를 만들 수
        있기 때문이다(TODO.md 21번).
        """
        now = time.monotonic()
        for unit in UNITS:
            with self.lock:
                mode = self.mode[unit]
                state = dict(self.drive[unit])
                roller_state = dict(self.roller.get(unit, {}))
            if self.rc_owns(unit):
                continue
            if mode == 'manual':
                stale = now - state['time'] > self.input_timeout
                twist = Twist()
                if not stale:
                    twist.linear.x = state['linear']
                    twist.angular.z = state['angular']
                self.cmd_pub[unit].publish(twist)
            # 컨베이어는 주행 모드와 별개 축이라 mode == 'manual' 여부와
            # 무관하게 계속 재발행한다 — cargo_load.py가 자동 미션
            # idle/stop일 때만 이 값을 채택하므로 자동과는 안 부딪힌다.
            if unit in self.roller_pub:
                roller_stale = now - roller_state.get('time', 0.0) > self.input_timeout
                value = Float32()
                if not roller_stale:
                    value.data = roller_state.get('value', 0.0)
                self.roller_pub[unit].publish(value)

    def set_mode(self, unit, mode):
        """UI 의 모드 이름을 호기가 아는 값으로 옮긴다."""
        spec = UNITS[unit]
        if mode not in spec['modes'] + ['idle']:
            return None
        if spec['mode_style'] == 'source' and mode == 'auto':
            # 1호기는 계절에 따라 중재 소스가 다르다.
            section = self.value(unit, 'section') or 'spring'
            wire = UNIT1_AUTO.get(str(section).strip().lower(), 'comp')
        else:
            wire = mode
        with self.lock:
            self.mode[unit] = mode
            self.drive[unit] = {'linear': 0.0, 'angular': 0.0, 'time': 0.0}
        self.mode_pub[unit].publish(String(data=wire))
        if mode != 'manual':
            self.cmd_pub[unit].publish(Twist())      # 수동에서 나올 때 0 한 번
        self.get_logger().info(f'unit{unit} mode -> {mode} (wire={wire})')
        return wire

    def set_drive(self, unit, linear, angular):
        with self.lock:
            self.drive[unit] = {
                'linear': max(-self.max_linear, min(self.max_linear, linear)),
                'angular': max(-self.max_angular, min(self.max_angular, angular)),
                'time': time.monotonic()}

    def set_roller(self, unit, value):
        if unit not in self.roller_pub:
            return False
        with self.lock:
            self.roller[unit] = {
                'value': max(-1.0, min(1.0, float(value))),
                'time': time.monotonic()}
        return True

    def set_solenoid(self, unit, locked):
        if unit not in self.solenoid_pub:
            return False
        self.solenoid_pub[unit].publish(Bool(data=bool(locked)))
        return True

    def set_marker_axes(self, unit, enabled):
        if unit not in self.axes_pub:
            return False
        self.axes_pub[unit].publish(Bool(data=bool(enabled)))
        return True

    def set_rc_enabled(self, unit, enabled):
        """웹/RC 조종 방식 토글 — 실제 반영 여부는 rc_bridge가
        rc/status.enabled로 되돌려주는 값을 snapshot()이 그대로 보여준다
        (marker/axes_enable과 같은 반영 패턴, 이 함수는 요청만 보낸다)."""
        if unit not in self.rc_enabled_pub:
            return False
        self.rc_enabled_pub[unit].publish(Bool(data=bool(enabled)))
        return True

    def set_estop(self, unit, stopped):
        if unit not in self.estop_pub:
            return False
        self.estop_pub[unit].publish(Bool(data=bool(stopped)))
        if not stopped:
            # 2호기 arbiter 는 mode_cmd 의 reset_emergency 로도 래치를 푼다.
            self.mode_pub[unit].publish(String(data='reset_emergency'))
        return True

    def send_mission(self, unit, action, roller=None):
        if unit not in self.mission_pub or action not in MISSION_ACTIONS:
            return False
        payload = {'action': action}
        if action == 'load':
            payload['roller'] = float(roller if roller is not None else 0.0)
        self.mission_pub[unit].publish(
            String(data=json.dumps(payload, separators=(',', ':'))))
        return True

    # ---------- 조회 ----------

    def value(self, unit, key):
        with self.lock:
            entry = self.feeds[unit].get(key)
        return entry[0] if entry else None

    def rc_owns(self, unit):
        """RC가 지금 이 호기의 전권을 쥐고 있는가 (TODO.md 21번).

        rc_bridge가 되돌려주는 ``rc/status.enabled``만 근거로 삼는다 —
        콘솔 자신이 ``rc/enabled``로 보낸 "요청"은 실제로 반영됐다는
        보장이 없어 안 쓴다(``set_rc_enabled`` 주석과 같은 원칙).
        ``stale_seconds`` 안의 최신 값일 때만 신뢰한다 — rc_bridge가
        죽어 피드백이 끊기면 자동으로 "전권 아님"으로 풀려야 웹이 다시
        조종할 수 있다(락 영구 고착 방지, 안전 우선 기본값은 웹).
        """
        with self.lock:
            entry = self.feeds[unit].get('rc')
        if entry is None:
            return False
        payload, stamp = entry
        if time.monotonic() - stamp > self.stale:
            return False
        return bool(isinstance(payload, dict) and payload.get('enabled'))

    def snapshot(self, unit):
        now = time.monotonic()
        with self.lock:
            feeds = {key: {'data': payload, 'age': round(now - stamp, 2)}
                     for key, (payload, stamp) in self.feeds[unit].items()}
            mode = self.mode[unit]
        camera_age = now - self.camera_time[unit] if self.camera_time[unit] else None
        drive = feeds.get('drive', {}).get('data') or {}
        motor_fresh = 'motor' in feeds and feeds['motor']['age'] <= self.stale
        drive_fresh = 'drive' in feeds and feeds['drive']['age'] <= self.stale
        camera_fresh = camera_age is not None and camera_age <= self.stale

        return {
            'unit': unit,
            'name': UNITS[unit]['name'],
            'modes': UNITS[unit]['modes'],
            # 모터 없이 카메라/marker_vision만 켜서 확인할 때도(오늘처럼)
            # 로봇 전체가 오프라인이라고 나오지 않게 한다.
            'online': bool(motor_fresh or drive_fresh or camera_fresh),
            'motor_online': motor_fresh,
            'mode': drive.get('mode', mode),
            'requested_mode': mode,
            'season': drive.get('season'),
            'source': drive.get('source'),
            'phase': drive.get('phase'),
            'action': drive.get('action'),
            # 1호기가 안 잡히면 UI 가 추종 버튼을 잠근다.
            'follow_available': bool(drive.get('fleet_link', False)),
            # rc 모드는 follow와 달리 잠그지 않는다(manual과 동일) — 이건
            # 순수 표시용 링크 상태다.
            'rc_available': bool(drive.get('rc_link', False)),
            'has_rc': unit in self.rc_enabled_pub,
            # rc_bridge의 rc/status를 그대로 반영한다(콘솔 자체 기억이
            # 아니다 — marker/status.axes_enabled와 같은 반영 패턴).
            'rc_enabled': bool((feeds.get('rc', {}).get('data') or {}).get(
                'enabled', False)),
            # rc_enabled와 달리 stale_seconds 안의 최신 값일 때만 참이다 —
            # 웹 쪽 조작 잠금(unit.html)은 반드시 이 값을 봐야 rc_bridge가
            # 죽었을 때도 자동으로 풀린다(rc_owns() 문서 참고, TODO.md 21번).
            'rc_owns': self.rc_owns(unit),
            'rc_robot_selected': bool((feeds.get('rc', {}).get('data') or {}).get(
                'robot_selected', False)),
            'rc_run_active': bool((feeds.get('rc', {}).get('data') or {}).get(
                'run_active', False)),
            'safety_stop': drive.get('safety_stop'),
            'emergency_stop': drive.get('emergency_stop'),
            'has_estop': UNITS[unit]['estop_topic'] is not None,
            'has_mission': UNITS[unit]['mission_topic'] is not None,
            'has_solenoid': unit in self.solenoid_pub,
            'has_roller': unit in self.roller_pub,
            'has_marker_axes': unit in self.axes_pub,
            'camera_age': None if camera_age is None else round(camera_age, 2),
            'camera_source': self.camera_source[unit],
            'feeds': feeds,
        }


# ---------------- 웹 ----------------

def unit_or_404(number):
    try:
        unit = int(number)
    except (TypeError, ValueError):
        return None
    return unit if unit in UNITS else None


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/unit/<number>')
def unit_page(number):
    unit = unit_or_404(number)
    if unit is None:
        return '알 수 없는 호기', 404
    return render_template('unit.html', unit=unit, name=UNITS[unit]['name'],
                           modes=UNITS[unit]['modes'])


@app.get('/api/units')
def api_units():
    return jsonify([ros_node.snapshot(unit) for unit in sorted(UNITS)])


@app.get('/api/unit/<number>')
def api_unit(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    return jsonify(ros_node.snapshot(unit))


def _rc_owns_response(unit, what):
    """RC 전권 중 웹 API를 막을 때 공통으로 쓰는 409 응답(TODO.md 21번)."""
    return jsonify({
        'ok': False,
        'error': f'RC가 조종 중입니다 — 웹에서 {what}하려면 먼저 "조종 방식"을 '
                 '웹으로 돌리세요',
    }), 409


@app.post('/api/unit/<number>/mode')
def api_mode(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    if ros_node.rc_owns(unit):
        return _rc_owns_response(unit, '모드를 바꾸')
    mode = (request.json or {}).get('mode', '')
    if mode == 'follow' and not ros_node.snapshot(unit)['follow_available']:
        # 1호기 /fleet/state 가 없으면 추종에 들어가지 않는다.
        return jsonify({'ok': False, 'error': '1호기가 잡히지 않는다'}), 409
    wire = ros_node.set_mode(unit, mode)
    if wire is None:
        return jsonify({'ok': False, 'error': 'unsupported mode'}), 400
    return jsonify({'ok': True, 'mode': mode, 'wire': wire})


@app.post('/api/unit/<number>/drive')
def api_drive(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    if ros_node.rc_owns(unit):
        return _rc_owns_response(unit, '주행하')
    payload = request.json or {}
    ros_node.set_drive(unit, float(payload.get('linear', 0.0)),
                       float(payload.get('angular', 0.0)))
    return jsonify({'ok': True})


@app.post('/api/unit/<number>/estop')
def api_estop(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    stopped = bool((request.json or {}).get('stop', True))
    if not ros_node.set_estop(unit, stopped):
        return jsonify({'ok': False, 'error': '이 호기에는 비상 정지 토픽이 없다'}), 400
    return jsonify({'ok': True, 'stop': stopped})


@app.post('/api/unit/<number>/solenoid')
def api_solenoid(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    if ros_node.rc_owns(unit):
        return _rc_owns_response(unit, '솔레노이드를 조작하')
    locked = bool((request.json or {}).get('lock', False))
    if not ros_node.set_solenoid(unit, locked):
        return jsonify({'ok': False, 'error': '이 호기에는 솔레노이드가 없다'}), 400
    return jsonify({'ok': True, 'lock': locked})


@app.post('/api/unit/<number>/roller')
def api_roller(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    if ros_node.rc_owns(unit):
        return _rc_owns_response(unit, '컨베이어를 조작하')
    payload = request.json or {}
    if not ros_node.set_roller(unit, float(payload.get('value', 0.0))):
        return jsonify({'ok': False, 'error': '이 호기에는 컨베이어가 없다'}), 400
    return jsonify({'ok': True})


@app.post('/api/unit/<number>/marker/axes')
def api_marker_axes(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    enabled = bool((request.json or {}).get('enabled', False))
    if not ros_node.set_marker_axes(unit, enabled):
        return jsonify({'ok': False, 'error': '이 호기에는 ArUco 인식이 없다'}), 400
    return jsonify({'ok': True, 'enabled': enabled})


@app.post('/api/unit/<number>/rc/enabled')
def api_rc_enabled(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    enabled = bool((request.json or {}).get('enabled', False))
    if not ros_node.set_rc_enabled(unit, enabled):
        return jsonify({'ok': False, 'error': '이 호기에는 RC 브리지가 없다'}), 400
    return jsonify({'ok': True, 'enabled': enabled})


@app.post('/api/unit/<number>/mission')
def api_mission(number):
    unit = unit_or_404(number)
    if unit is None:
        return jsonify({'error': 'unknown unit'}), 404
    payload = request.json or {}
    if not ros_node.send_mission(unit, str(payload.get('action', '')),
                                 payload.get('roller')):
        return jsonify({'ok': False, 'error': 'unsupported action'}), 400
    return jsonify({'ok': True})


@app.get('/api/unit/<number>/camera.mjpg')
def api_camera(number):
    unit = unit_or_404(number)
    if unit is None:
        return '알 수 없는 호기', 404

    def generate():
        while not shutdown.is_set():
            frame = ros_node.cameras[unit].wait(timeout=2.0)
            if frame is None:
                continue
            yield (b'--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: '
                   + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
    return Response(stream_with_context(generate()),
                    mimetype='multipart/x-mixed-replace; boundary=FRAME')


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = Console()
    host = str(ros_node.get_parameter('host').value)
    port = int(ros_node.get_parameter('port').value)

    threading.Thread(
        target=lambda: app.run(host=host, port=port, threaded=True, use_reloader=False),
        daemon=True).start()
    ros_node.get_logger().info(f'console web on http://{host}:{port}/')

    try:
        rclpy.spin(ros_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        shutdown.set()
        try:
            for unit in UNITS:            # 나갈 때는 모든 호기에 0 을 남긴다
                ros_node.cmd_pub[unit].publish(Twist())
        except Exception:
            pass
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
