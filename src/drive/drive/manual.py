"""
키보드 텔레오퍼레이션 입력 노드.

터미널에서 WASD 로 로봇을 조종해 /cmd_vel/manual (Twist) 로 발행한다.
  w/s : 전진/후진   a/d : 좌/우 회전   space : 정지   q : 종료

시험용 입력 노드이며 최종 /cmd_vel 은 arbiter가 선택한다.
arbiter 가 이 후보를 고르려면 모드가 manual 이어야 한다.
1호기 없이 이 호기만으로 주행할 때는 mars_launch/manual.launch.py 를 쓴다.
노드는 호기 네임스페이스 안에서 떠야 한다 (--ros-args -r __ns:=/unit2).

.. warning::
   **이 노드는 실제 터미널(TTY)에서 ``ros2 run`` 으로 띄워야 한다.**
   launch 파일에 넣으면 stdin이 터미널이 아니라 raw 모드 설정이 실패한다.
   그래서 manual.launch.py 는 모터와 arbiter만 올리고 이 노드는 뺐다.

값은 drive/config/arbiter.yaml 의 ``manual`` 구역에 있다. ``ros2 run`` 으로
띄울 때는 yaml이 실리지 않으므로 아래 declare_parameter 기본값이 쓰인다.
바꾸려면 ``--ros-args -p max_linear:=0.2`` 로 덮어쓴다.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CMD_TOPIC = "cmd_vel/manual"

HELP = """
Keyboard Teleop -> cmd_vel/manual
  w/s : 전진/후진
  a/d : 좌/우 회전
  space : 즉시 정지
  q : 종료
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('manual')

        # 조작 단위와 상한은 drive/config/arbiter.yaml의 manual 구역에서 조정한다.
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 0.50)
        self.declare_parameter('repeat_rate_hz', 20.0)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.period = 1.0 / max(1.0, float(self.get_parameter('repeat_rate_hz').value))

        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.linear = 0.0
        self.angular = 0.0
        self.get_logger().info(
            f'Keyboard Teleop started -> {CMD_TOPIC} '
            f'(max {self.max_linear:.2f} m/s, {self.max_angular:.2f} rad/s)')
        print(HELP)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

    def _apply(self, key):
        """키 하나를 반영한다. 종료 키면 False."""
        if key == 'w':
            self.linear = self.clamp(self.linear + self.linear_step, self.max_linear)
        elif key == 's':
            self.linear = self.clamp(self.linear - self.linear_step, self.max_linear)
        elif key == 'a':
            self.angular = self.clamp(self.angular + self.angular_step, self.max_angular)
        elif key == 'd':
            self.angular = self.clamp(self.angular - self.angular_step, self.max_angular)
        elif key == ' ':
            self.linear = self.angular = 0.0
        elif key in ('q', '\x03'):      # q 또는 Ctrl-C
            return False
        return True

    def spin_keyboard(self):
        """키를 읽으며 repeat_rate_hz 로 계속 재발행한다.

        키를 안 눌러도 계속 발행해야 arbiter의 source_timeout_s(기본 0.5초)에
        걸려 끊기지 않는다. 반대로 이 노드가 죽으면 발행이 멈추고 arbiter가
        스스로 0을 내보내므로, 그것이 이 경로의 안전장치다.
        """
        fd = sys.stdin.fileno()
        if not sys.stdin.isatty():
            self.get_logger().error(
                'stdin이 터미널이 아니다. 이 노드는 launch가 아니라 '
                '터미널에서 "ros2 run drive manual" 로 실행해야 한다.')
            return

        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ready, _, _ = select.select([sys.stdin], [], [], self.period)
                if ready and not self._apply(sys.stdin.read(1)):
                    break
                self._publish()
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.linear = self.angular = 0.0
            self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.spin_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
