#!/usr/bin/env python3
"""1호기 없이 2호기 추종(follow) 모드를 벤치에서 시험하기 위한 가짜 리더.

fleet/follower.py는 ``/fleet/state``가 살아 있어야만
(``fleet.follower.Follower._on_state``) summer가 아닌 계절에서 자동으로
``follow`` 액션을 걸고, drive/arbiter도 ``/fleet/state``가 최근에 왔을
때만(``fleet_alive()``) ``mode_cmd=follow``를 받아들인다. 실물 1호기 없이
2호기 단독으로 추종·주행·컨베이어·솔레노이드·ArUco 인식을 한 번에
시험하려면 이 신호를 대신 내주는 노드가 필요하다 -- 그게 이 노드다.

**절대 실물 1호기와 같은 ROS_DOMAIN_ID에서 같이 켜지 않는다.** 두
발행자가 동시에 ``/fleet/state``를 쏘면 값이 서로 덮어써 예측 불가능하게
동작한다. web_manual.launch.py는 기본적으로 격리된 test_domain(기본
77)에서 돌아 실물 운용 도메인(10)과 분리돼 있으므로 정상적인 사용에서는
충돌하지 않는다.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fleet import protocol


class FakeLeader(Node):
    def __init__(self):
        super().__init__('fleet_fake_leader')

        self.declare_parameter('section', 'spring')
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('rate_hz', 5.0)
        self.section = str(self.get_parameter('section').value).strip().lower()
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        if self.section not in protocol.SECTIONS:
            raise ValueError(
                f'unknown section {self.section!r}; choose one of '
                f'{sorted(protocol.SECTIONS)}')
        if self.mode not in {'auto', 'manual'}:
            raise ValueError(f'unknown mode {self.mode!r}')

        self.pub = self.create_publisher(String, protocol.TOPIC_STATE, 10)
        rate = max(0.5, float(self.get_parameter('rate_hz').value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().warn(
            f'가짜(FAKE) 1호기 /fleet/state 발행 중 (section={self.section}, '
            f'mode={self.mode}) -- 실물 1호기와 같은 ROS_DOMAIN_ID에서 '
            '절대 같이 켜지 말 것. 벤치 시험 전용.')

    def _tick(self):
        msg = String()
        msg.data = protocol.message(
            section=self.section, mode=self.mode, leader_stopped=False,
            source='fake_leader')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLeader()
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
