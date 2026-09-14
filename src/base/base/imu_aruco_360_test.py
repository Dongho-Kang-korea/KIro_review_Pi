"""One-shot live check of a 360 degree IMU turn against an ArUco reference.

The robot is commanded through ``cmd_vel/manual`` so the normal drive arbiter,
safety stop and motor command timeout remain in the path.  This node is inert
unless the ``armed`` parameter is explicitly set to true.

Sequence:

1. Put the arbiter in idle and collect distinct, valid ArUco pose frames.
2. Put the arbiter in manual and turn CCW once while unwrapping IMU heading.
3. Stop after the accumulated IMU angle reaches 360 degrees.
4. Reacquire the same ArUco marker and measure its yaw error from step 1.
5. Rotate by the marker error until its yaw matches the stored value, while
   independently accumulating and reporting the additional IMU rotation.
"""
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, String


def angle_error_deg(current, target):
    """Signed shortest angle from target to current, in [-180, 180)."""
    return (float(current) - float(target) + 180.0) % 360.0 - 180.0


def circular_mean_deg(values):
    """Circular mean that remains correct around the -180/180 boundary."""
    if not values:
        raise ValueError('at least one angle is required')
    sin_mean = statistics.fmean(math.sin(math.radians(value)) for value in values)
    cos_mean = statistics.fmean(math.cos(math.radians(value)) for value in values)
    if abs(sin_mean) < 1e-12 and abs(cos_mean) < 1e-12:
        raise ValueError('circular mean is undefined')
    return math.degrees(math.atan2(sin_mean, cos_mean))


def alignment_angular_rps(error_deg, kp, minimum, maximum):
    """Match coupling.py's marker-yaw steering sign and clamp its speed."""
    if abs(error_deg) < 1e-12:
        return 0.0
    magnitude = min(abs(float(maximum)), abs(float(kp) * float(error_deg)))
    magnitude = max(abs(float(minimum)), magnitude)
    return -math.copysign(magnitude, float(error_deg))


class HeadingAccumulator:
    """Accumulate signed IMU heading across any number of 0/360 wraps."""

    def __init__(self, heading_deg, max_step_deg=45.0):
        self.last = float(heading_deg) % 360.0
        self.total_deg = 0.0
        self.max_step_deg = abs(float(max_step_deg))

    def update(self, heading_deg):
        current = float(heading_deg) % 360.0
        step = angle_error_deg(current, self.last)
        if abs(step) > self.max_step_deg:
            raise ValueError(
                f'IMU heading jump {step:+.2f} deg exceeds '
                f'{self.max_step_deg:.2f} deg')
        self.total_deg += step
        self.last = current
        return step


class ImuAruco360Test(Node):
    def __init__(self):
        super().__init__('imu_aruco_360_test')

        parameters = {
            # Must be explicitly enabled on the command line.  With false this
            # node never emits a non-zero velocity.
            'armed': False,
            'marker_id': 1,
            'initial_marker_samples': 8,
            'post_turn_marker_samples': 6,
            'sensor_timeout_s': 0.8,
            'prepare_timeout_s': 15.0,
            'mode_timeout_s': 4.0,
            'full_turn_target_deg': 360.0,
            'full_turn_max_rps': 0.30,
            'full_turn_min_rps': 0.07,
            'full_turn_kp_rps_per_deg': 0.006,
            'full_turn_timeout_s': 35.0,
            'wrong_direction_limit_deg': 15.0,
            'max_imu_step_deg': 45.0,
            'settle_s': 1.0,
            'reacquire_timeout_s': 10.0,
            'marker_yaw_tolerance_deg': 2.0,
            'align_kp_rps_per_deg': 0.012,
            'align_min_rps': 0.05,
            'align_max_rps': 0.18,
            'align_hold_s': 0.6,
            'align_hold_frames': 3,
            'align_timeout_s': 15.0,
            'publish_rate_hz': 20.0,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self.armed = bool(self.get_parameter('armed').value)
        self.phase = 'prepare_idle' if self.armed else 'not_armed'
        self.reason = '' if self.armed else 'armed_parameter_is_false'
        self.phase_started = time.monotonic()
        self.finished = False
        self.success = False
        self.exit_at = None

        self.drive_mode = None
        self.drive_status_time = None
        self.safety_stop = False
        self.emergency_stop = False

        self.imu_heading_deg = None
        self.imu_time = None
        self.full_turn_accumulator = None
        self.alignment_accumulator = None

        self.marker_yaw_deg = None
        self.marker_time = None
        self.marker_frame_seq = None
        self.marker_id = None
        self.initial_samples = []
        self.post_turn_samples = []
        self.align_good_frames = 0
        self.align_good_since = None

        self.initial_marker_yaw_deg = None
        self.post_turn_marker_yaw_deg = None
        self.marker_difference_after_turn_deg = None
        self.final_marker_yaw_deg = None
        self.final_marker_difference_deg = None
        self.full_turn_imu_deg = None
        self.additional_alignment_imu_deg = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/manual', 10)
        self.mode_pub = self.create_publisher(String, 'drive/mode_cmd', 10)
        status_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, 'test/imu_aruco_360/status', status_qos)

        self.create_subscription(
            Float32, 'imu/heading_deg', self._on_heading, 10)
        self.create_subscription(String, 'marker/status', self._on_marker, 10)
        self.create_subscription(String, 'drive/status', self._on_drive, 10)
        self.create_subscription(Bool, 'safety/stop', self._on_safety, 10)
        self.create_subscription(
            Bool, 'safety/emergency_stop', self._on_emergency, 10)

        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'IMU/ArUco 360 test ready | '
            f'armed={self.armed} | marker_id={self._parameter("marker_id")}')
        if not self.armed:
            self.get_logger().error(
                'Motion is disarmed. Run with --ros-args -p armed:=true.')

    def _parameter(self, name):
        return self.get_parameter(name).value

    def _transition(self, phase):
        self.phase = phase
        self.phase_started = time.monotonic()
        self.get_logger().info(f'TEST PHASE -> {phase}')

    def _publish_mode(self, mode):
        self.mode_pub.publish(String(data=mode))

    def _publish_cmd(self, angular=0.0):
        cmd = Twist()
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def _imu_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (
            self.imu_heading_deg is not None and self.imu_time is not None and
            now - self.imu_time <= float(self._parameter('sensor_timeout_s')))

    def _marker_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (
            self.marker_yaw_deg is not None and self.marker_time is not None and
            now - self.marker_time <= float(self._parameter('sensor_timeout_s')))

    def _on_heading(self, msg):
        heading = float(msg.data) % 360.0
        if not math.isfinite(heading):
            return
        self.imu_heading_deg = heading
        self.imu_time = time.monotonic()
        try:
            if self.phase == 'full_turn' and self.full_turn_accumulator is not None:
                self.full_turn_accumulator.update(heading)
            elif self.phase == 'align' and self.alignment_accumulator is not None:
                self.alignment_accumulator.update(heading)
        except ValueError as exc:
            self._fail('imu_heading_jump', str(exc))

    def _on_marker(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        frame_seq = value.get('frame_seq')
        if (not isinstance(frame_seq, int) or isinstance(frame_seq, bool) or
                frame_seq == self.marker_frame_seq):
            return
        self.marker_frame_seq = frame_seq

        marker_id = value.get('id')
        yaw = value.get('yaw_deg')
        valid = (
            value.get('enabled', False) and
            value.get('detector_ready', False) and
            value.get('camera_frame_fresh', False) and
            value.get('pose_available', False) and
            value.get('detected', False) and
            not value.get('error') and
            marker_id == int(self._parameter('marker_id')) and
            isinstance(yaw, (int, float)) and not isinstance(yaw, bool) and
            math.isfinite(float(yaw)))
        if not valid:
            return

        self.marker_id = int(marker_id)
        self.marker_yaw_deg = float(yaw)
        self.marker_time = time.monotonic()
        if self.phase == 'capture_initial':
            self.initial_samples.append(self.marker_yaw_deg)
        elif self.phase == 'reacquire':
            self.post_turn_samples.append(self.marker_yaw_deg)
        elif self.phase == 'align':
            error = angle_error_deg(
                self.marker_yaw_deg, self.initial_marker_yaw_deg)
            if abs(error) <= float(
                    self._parameter('marker_yaw_tolerance_deg')):
                if self.align_good_since is None:
                    self.align_good_since = self.marker_time
                    self.align_good_frames = 0
                self.align_good_frames += 1
            else:
                self.align_good_since = None
                self.align_good_frames = 0

    def _on_drive(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.drive_mode = str(value.get('mode', ''))
        self.drive_status_time = time.monotonic()
        self.safety_stop = bool(value.get('safety_stop', self.safety_stop))
        self.emergency_stop = bool(
            value.get('emergency_stop', self.emergency_stop))

    def _on_safety(self, msg):
        self.safety_stop = bool(msg.data)

    def _on_emergency(self, msg):
        self.emergency_stop = bool(msg.data)

    def _result(self):
        return {
            'success': self.success,
            'phase': self.phase,
            'reason': self.reason,
            'marker_id': self.marker_id,
            'initial_marker_yaw_deg': self.initial_marker_yaw_deg,
            'post_turn_marker_yaw_deg': self.post_turn_marker_yaw_deg,
            'marker_difference_after_turn_deg':
                self.marker_difference_after_turn_deg,
            'full_turn_imu_deg': self.full_turn_imu_deg,
            'additional_alignment_imu_deg':
                self.additional_alignment_imu_deg,
            'additional_alignment_imu_abs_deg': (
                abs(self.additional_alignment_imu_deg)
                if self.additional_alignment_imu_deg is not None else None),
            'final_marker_yaw_deg': self.final_marker_yaw_deg,
            'final_marker_difference_deg': self.final_marker_difference_deg,
            'drive_mode': self.drive_mode,
            'imu_heading_deg': self.imu_heading_deg,
        }

    def _publish_status(self):
        self.status_pub.publish(String(
            data=json.dumps(self._result(), ensure_ascii=False)))

    def _finish(self):
        if self.finished:
            return
        self.success = True
        self.phase = 'complete'
        self.reason = ''
        self.final_marker_yaw_deg = self.marker_yaw_deg
        self.final_marker_difference_deg = angle_error_deg(
            self.final_marker_yaw_deg, self.initial_marker_yaw_deg)
        self.additional_alignment_imu_deg = (
            self.alignment_accumulator.total_deg
            if self.alignment_accumulator is not None else 0.0)
        self.finished = True
        self.exit_at = time.monotonic() + 1.0
        self._publish_cmd()
        self._publish_mode('idle')
        result = json.dumps(self._result(), ensure_ascii=False, indent=2)
        self.get_logger().info('TEST COMPLETE\n' + result)
        self._publish_status()

    def _fail(self, reason, detail=''):
        if self.finished:
            return
        self.success = False
        self.phase = 'error'
        self.reason = reason if not detail else f'{reason}: {detail}'
        if self.full_turn_accumulator is not None:
            self.full_turn_imu_deg = self.full_turn_accumulator.total_deg
        if self.alignment_accumulator is not None:
            self.additional_alignment_imu_deg = (
                self.alignment_accumulator.total_deg)
        self.finished = True
        self.exit_at = time.monotonic() + 1.0
        self._publish_cmd()
        self._publish_mode('idle')
        result = json.dumps(self._result(), ensure_ascii=False, indent=2)
        self.get_logger().error('TEST FAILED\n' + result)
        self._publish_status()

    def _tick(self):
        now = time.monotonic()
        self._publish_status()

        if self.finished:
            self._publish_cmd()
            self._publish_mode('idle')
            return
        if not self.armed:
            self._fail('not_armed')
            return
        if self.safety_stop or self.emergency_stop:
            self._fail('safety_stop')
            return

        elapsed = now - self.phase_started
        if self.phase == 'prepare_idle':
            self._publish_cmd()
            self._publish_mode('idle')
            if self.drive_mode == 'idle' and self._imu_fresh(now):
                self.initial_samples.clear()
                self._transition('capture_initial')
            elif elapsed > float(self._parameter('prepare_timeout_s')):
                self._fail('prepare_timeout')

        elif self.phase == 'capture_initial':
            self._publish_cmd()
            self._publish_mode('idle')
            required = int(self._parameter('initial_marker_samples'))
            if len(self.initial_samples) >= required:
                self.initial_marker_yaw_deg = circular_mean_deg(
                    self.initial_samples[-required:])
                self.get_logger().info(
                    'Initial ArUco yaw stored: '
                    f'{self.initial_marker_yaw_deg:+.3f} deg '
                    f'({required} distinct frames)')
                self._transition('select_manual')
            elif elapsed > float(self._parameter('prepare_timeout_s')):
                self._fail('initial_marker_timeout')

        elif self.phase == 'select_manual':
            self._publish_cmd()
            self._publish_mode('manual')
            if self.drive_mode == 'manual' and self._imu_fresh(now):
                self.full_turn_accumulator = HeadingAccumulator(
                    self.imu_heading_deg,
                    self._parameter('max_imu_step_deg'))
                self._transition('full_turn')
            elif elapsed > float(self._parameter('mode_timeout_s')):
                self._fail('manual_mode_timeout')

        elif self.phase == 'full_turn':
            if not self._imu_fresh(now):
                self._fail('imu_timeout_during_full_turn')
                return
            total = self.full_turn_accumulator.total_deg
            target = float(self._parameter('full_turn_target_deg'))
            if total < -float(self._parameter('wrong_direction_limit_deg')):
                self._fail('wrong_turn_direction')
            elif total >= target:
                self.full_turn_imu_deg = total
                self._publish_cmd()
                self.get_logger().info(
                    f'Full CCW turn detected by IMU: {total:+.3f} deg')
                self._transition('settle')
            elif elapsed > float(self._parameter('full_turn_timeout_s')):
                self._fail('full_turn_timeout')
            else:
                remaining = max(0.0, target - total)
                speed = min(
                    float(self._parameter('full_turn_max_rps')),
                    float(self._parameter('full_turn_kp_rps_per_deg')) *
                    remaining)
                speed = max(
                    float(self._parameter('full_turn_min_rps')), speed)
                self._publish_cmd(abs(speed))  # ROS angular.z > 0 is CCW.

        elif self.phase == 'settle':
            self._publish_cmd()
            if not self._imu_fresh(now):
                self._fail('imu_timeout_while_settling')
            elif elapsed >= float(self._parameter('settle_s')):
                self.post_turn_samples.clear()
                self._transition('reacquire')

        elif self.phase == 'reacquire':
            self._publish_cmd()
            required = int(self._parameter('post_turn_marker_samples'))
            if len(self.post_turn_samples) >= required:
                self.post_turn_marker_yaw_deg = circular_mean_deg(
                    self.post_turn_samples[-required:])
                self.marker_difference_after_turn_deg = angle_error_deg(
                    self.post_turn_marker_yaw_deg,
                    self.initial_marker_yaw_deg)
                self.get_logger().info(
                    'ArUco after full turn: '
                    f'{self.post_turn_marker_yaw_deg:+.3f} deg | '
                    'difference from initial: '
                    f'{self.marker_difference_after_turn_deg:+.3f} deg')
                self.alignment_accumulator = HeadingAccumulator(
                    self.imu_heading_deg,
                    self._parameter('max_imu_step_deg'))
                if abs(self.marker_difference_after_turn_deg) <= float(
                        self._parameter('marker_yaw_tolerance_deg')):
                    self._finish()
                else:
                    self.align_good_since = None
                    self.align_good_frames = 0
                    self._transition('align')
            elif elapsed > float(self._parameter('reacquire_timeout_s')):
                self._fail('aruco_reacquire_timeout')

        elif self.phase == 'align':
            if not self._imu_fresh(now):
                self._fail('imu_timeout_during_alignment')
            elif not self._marker_fresh(now):
                self._publish_cmd()
                if elapsed > float(self._parameter('align_timeout_s')):
                    self._fail('aruco_timeout_during_alignment')
            elif elapsed > float(self._parameter('align_timeout_s')):
                self._fail('alignment_timeout')
            else:
                error = angle_error_deg(
                    self.marker_yaw_deg, self.initial_marker_yaw_deg)
                held = (
                    self.align_good_since is not None and
                    now - self.align_good_since >= float(
                        self._parameter('align_hold_s')) and
                    self.align_good_frames >= int(
                        self._parameter('align_hold_frames')))
                if held:
                    self._publish_cmd()
                    self._finish()
                elif abs(error) <= float(
                        self._parameter('marker_yaw_tolerance_deg')):
                    self._publish_cmd()
                else:
                    angular = alignment_angular_rps(
                        error,
                        self._parameter('align_kp_rps_per_deg'),
                        self._parameter('align_min_rps'),
                        self._parameter('align_max_rps'))
                    self._publish_cmd(angular)

    def ready_to_exit(self):
        return self.finished and self.exit_at is not None and (
            time.monotonic() >= self.exit_at)


def main(args=None):
    rclpy.init(args=args)
    node = ImuAruco360Test()
    try:
        while rclpy.ok() and not node.ready_to_exit():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._fail('operator_cancelled')
    finally:
        node._publish_cmd()
        node._publish_mode('idle')
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
