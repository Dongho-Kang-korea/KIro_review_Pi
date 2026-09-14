"""BNO086 방위각을 발행하며 미장착 상태도 보고한다."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from std_msgs.msg import Float32, String


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu')

        # BNO086 장착 후 base/config/imu.yaml에서 사용 여부와 영점을 설정한다.
        self.declare_parameter('enabled', False)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('heading_offset_deg', 0.0)
        self.declare_parameter('i2c_address', 0x4B)
        self.declare_parameter('reconnect_interval_s', 1.0)
        # true  = ROTATION_VECTOR(자이로+가속도+지자기). 절대 북쪽이 있지만
        #         지자기 융합이 재수렴하면서 절대 heading 이 몇 도씩 튄다.
        # false = GAME_ROTATION_VECTOR(자이로+가속도, 지자기 없음).
        #         절대 방위는 없고 부팅 자세가 기준이지만, 상대 회전이
        #         지자기 재기준에 흔들리지 않는다 — 탱크턴 각도 제어용.
        # 2026-08-31 360도 시험: true 에서 사이클당 절대 heading 이 +4.29도
        # 씩 밀렸다(회전 자체는 360.43+-0.31 로 정확). 그래서 false 로 시험한다.
        self.declare_parameter('use_magnetometer', True)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.offset = float(self.get_parameter('heading_offset_deg').value)
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.reconnect_interval = max(0.1, float(
            self.get_parameter('reconnect_interval_s').value))
        self.use_magnetometer = bool(
            self.get_parameter('use_magnetometer').value)
        self.next_open_at = 0.0
        self.sensor = None
        self.error = ''
        if self.enabled:
            self._open()
        self.heading_pub = self.create_publisher(Float32, 'imu/heading_deg', 10)
        self.quaternion_pub = self.create_publisher(
            Quaternion, 'imu/quaternion', 10)
        self.rpy_pub = self.create_publisher(String, 'imu/rpy_deg', 10)
        self.status_pub = self.create_publisher(String, 'imu/status', 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

    def _open(self):
        self.sensor = None
        try:
            import board
            import busio
            from adafruit_bno08x import (
                BNO_REPORT_GAME_ROTATION_VECTOR,
                BNO_REPORT_ROTATION_VECTOR,
            )
            from adafruit_bno08x.i2c import BNO08X_I2C
            self.sensor = BNO08X_I2C(
                busio.I2C(board.SCL, board.SDA),
                address=self.i2c_address)
            self.next_open_at = 0.0
            report = (BNO_REPORT_ROTATION_VECTOR if self.use_magnetometer
                      else BNO_REPORT_GAME_ROTATION_VECTOR)
            # 리포트마다 읽는 속성이 다르다. enable_feature 만 바꾸고
            # .quaternion 을 그대로 읽으면 값이 안 들어온다.
            self.quaternion_attr = ('quaternion' if self.use_magnetometer
                                    else 'game_quaternion')
            self.get_logger().info(
                'BNO086 report: '
                + ('ROTATION_VECTOR (지자기 융합)' if self.use_magnetometer
                   else 'GAME_ROTATION_VECTOR (지자기 제외)'))
            try:
                self.sensor.enable_feature(report)
            except RuntimeError as exc:
                # 전 담당자 실기 기록과 동일하게 초기 SHTP batch 경고는
                # 연결 실패로 취급하지 않고 이후 Quaternion 읽기를 계속한다.
                self.error = f'Initial packet warning: {exc}'
                self.get_logger().warn(self.error)
        except Exception as exc:
            self.sensor = None
            self.next_open_at = time.monotonic() + self.reconnect_interval
            self.error = str(exc)
            self.get_logger().warn(f'BNO086 unavailable: {exc}')

    def _tick(self):
        heading = None
        roll = pitch = yaw = None
        if (self.enabled and self.sensor is None and
                time.monotonic() >= self.next_open_at):
            self._open()
        if self.sensor is not None:
            try:
                x, y, z, w = getattr(self.sensor, self.quaternion_attr)
                quaternion_msg = Quaternion()
                quaternion_msg.x = float(x)
                quaternion_msg.y = float(y)
                quaternion_msg.z = float(z)
                quaternion_msg.w = float(w)
                self.quaternion_pub.publish(quaternion_msg)

                roll_rad = math.atan2(
                    2.0 * (w * x + y * z),
                    1.0 - 2.0 * (x * x + y * y))
                pitch_sin = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
                pitch_rad = math.asin(pitch_sin)
                yaw_rad = math.atan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z))
                roll = math.degrees(roll_rad)
                pitch = math.degrees(pitch_rad)
                yaw = math.degrees(yaw_rad)
                heading = (yaw + self.offset) % 360.0

                rpy_msg = String()
                rpy_msg.data = (
                    f'ROLL {roll:+7.2f} deg | PITCH {pitch:+7.2f} deg | '
                    f'YAW {yaw:+7.2f} deg')
                self.rpy_pub.publish(rpy_msg)
                self.error = ''
                msg = Float32(); msg.data = float(heading)
                self.heading_pub.publish(msg)
            except Exception as exc:
                self.error = str(exc)
        status = String()
        status.data = json.dumps({
            'enabled': self.enabled, 'ready': heading is not None,
            'heading_deg': heading, 'error': self.error,
            'roll_deg': roll, 'pitch_deg': pitch, 'yaw_deg': yaw,
            'use_magnetometer': self.use_magnetometer,
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args); node = ImuNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
