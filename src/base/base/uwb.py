"""DWM1001 거리를 발행하며 미장착 상태도 보고한다."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class UwbNode(Node):
    def __init__(self):
        super().__init__('uwb')

        # DWM1001 장착 후 base/config/uwb.yaml에서 포트와 거리 보정을 설정한다.
        self.declare_parameter('enabled', False)
        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('offset_mm', 0.0)
        self.declare_parameter('shell_enter_delay_s', 0.2)
        self.declare_parameter('data_timeout_s', 3.0)
        self.declare_parameter('reconnect_interval_s', 2.0)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.offset = float(self.get_parameter('offset_mm').value)
        self.serial = None
        self.last_mm = math.nan
        self.last_valid_time = None
        self.stream_started = None
        self.last_open_attempt = -math.inf
        self.error = ''
        if self.enabled:
            self._open()
        self.distance_pub = self.create_publisher(Float32, 'uwb/distance_mm', 10)
        self.status_pub = self.create_publisher(String, 'uwb/status', 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

    def _open(self):
        self.last_open_attempt = time.monotonic()
        try:
            import serial
            port = str(self.get_parameter('port').value)
            if not port:
                raise ValueError('serial port is empty')
            self._close()
            self.serial = serial.Serial(
                port, int(self.get_parameter('baudrate').value), timeout=0)
            self.serial.reset_input_buffer()
            # The two shell-entry CRs need a real gap. Sending b'\r\r' in one
            # USB write intermittently leaves the DWM1001 in binary TLV mode.
            delay = max(0.05, float(
                self.get_parameter('shell_enter_delay_s').value))
            self.serial.write(b'\r')
            self.serial.flush()
            time.sleep(delay)
            self.serial.write(b'\r')
            self.serial.flush()
            time.sleep(delay)
            self.serial.reset_input_buffer()
            # `lec` starts a continuous text stream; sending it at every 10 Hz
            # tick interrupts/floods the shell, so start it exactly once.
            self.serial.write(b'lec\r')
            self.serial.flush()
            self.last_mm = math.nan
            self.last_valid_time = None
            self.stream_started = time.monotonic()
            self.error = ''
        except Exception as exc:
            self._close()
            self.error = str(exc)
            self.get_logger().warn(f'UWB unavailable: {exc}')

    def _close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None

    @staticmethod
    def _parse(line):
        text = line.decode('utf-8', 'ignore').strip()
        try:
            value = json.loads(text)
            if isinstance(value, dict):
                return float(value['distance_mm'])
        except Exception:
            pass
        # DWM1001 셸 `lec` 응답: DIST,<n>,AN0,<라벨>,x,y,z,거리(m) — 셸 에코/
        # 프롬프트 라인은 콤마가 없어 여기서 걸러진다. 마지막 필드는 미터라
        # mm로 환산한다(실측: `DIST,1,AN0,D00C,0.00,0.00,0.00,0.79` = 0.79m).
        if ',' not in text:
            return None
        try:
            return float(text.split(',')[-1]) * 1000.0
        except ValueError:
            return None

    def _tick(self):
        now = time.monotonic()
        if (self.enabled and self.serial is None and
                now - self.last_open_attempt >= float(
                    self.get_parameter('reconnect_interval_s').value)):
            self._open()
        if self.serial is not None:
            try:
                while self.serial.in_waiting:
                    line = self.serial.readline()
                    if not line:
                        continue
                    parsed = self._parse(line)
                    if parsed is not None:
                        self.last_mm = parsed + self.offset
                        self.last_valid_time = time.monotonic()
                        self.error = ''
            except Exception as exc:
                self.error = str(exc)
                self._close()

        timeout = float(self.get_parameter('data_timeout_s').value)
        age_origin = self.last_valid_time or self.stream_started
        if (self.serial is not None and age_origin is not None and
                time.monotonic() - age_origin > timeout):
            self.error = 'distance stream timeout'
            self._close()

        now = time.monotonic()
        ready = (
            self.serial is not None and math.isfinite(self.last_mm) and
            self.last_valid_time is not None and
            now - self.last_valid_time <= timeout)
        if ready:
            msg = Float32(); msg.data = float(self.last_mm)
            self.distance_pub.publish(msg)
        status = String()
        status.data = json.dumps({
            'enabled': self.enabled, 'ready': ready,
            'distance_mm': self.last_mm if ready else None,
            'distance_age_s': (
                None if self.last_valid_time is None else
                max(0.0, now - self.last_valid_time)),
            'error': self.error,
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args); node = UwbNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node._close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
