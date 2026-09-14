"""결합한 상태로 1호기와 같은 속도로 굴러가는 협조 주행.

결합이 ``locked`` 된 뒤에는 ``coupling.py`` 가 속도를 놓는다(성공 후에도 계속
쥐고 있으면 솔레노이드를 풀기 전까지 못 움직인다). 그때부터 이 노드가
``cmd_vel/coupled`` 를 낸다.

**왜 추종(follow_leader)을 재사용하지 않는가**: 추종은 마커 거리를
``target_distance_mm`` 로 맞추는 제어다. 결합 중에는 결합 로드가 거리를
물리적으로 고정하므로, 거리 제어기가 이미 고정된 값을 맞추려 들며 로드와
싸운다. 그래서 결합 이동은 거리 제어가 아니라 **속도 복제**여야 한다.

**왜 그냥 끌려가지 않는가**: 주행축은 초기화 때 closed-loop 로 올라가 속도 0을
유지한다 — 명령을 안 주면 바퀴가 잠겨 1호기가 2호기를 끌어야 한다. 눈길·경사가
있는 제설 구간에서는 2호기도 자기 몫을 굴려야 한다. ``coupling.py`` 의
``verify_pull`` 이 2호기 모터로 당겨 전류를 보는 것도 같은 전제다.

**1호기가 보내줘야 하는 것**: ``/fleet/cmd_vel/unit2`` 에 실리는 ``Twist``.
1호기 ``drive/coupled.py`` 가 자기 구동 명령과 **같은 루프에서** 50Hz 로 낸다 —
두 호기가 같은 값을 같은 순간에 받게 하려는 것이라, 이걸 그대로 쓰는 것이
1호기 바퀴와 어긋나지 않는 유일한 방법이다.

**왜 /fleet/state 가 아닌가** (2026-09-04): 처음에는 ``/fleet/state`` JSON 의
``leader_linear`` 필드를 읽게 짰는데, 1호기는 그 필드를 아예 싣지 않는다
(``fleet/protocol.py:make_state`` 는 section/phase/leader_stopped/mode 뿐이다).
게다가 ``/fleet/state`` 는 1Hz 주기의 상태 알림용이라, 실려 있었더라도 결합
주행 명령을 나르기엔 20배 느리다. 두 호기가 서로 다른 채널로 말하고 있었다.

명령이 끊기면 0을 내고 ``reason`` 에 ``leader_velocity_missing`` /
``leader_velocity_stale`` 을 남긴다 — 안전 실패다.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String


def leader_qos():
    """1호기 ``coupled.py`` 의 remote_pub 과 짝을 맞춘 QoS.

    한쪽이라도 다르면 ROS 2 는 연결 자체를 안 맺고 아무 경고도 안 낸다.
    밀린 속도 명령을 뒤늦게 따라가면 위험하므로 최신 한 건만 본다.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)


def clamp(value, limit):
    return max(-abs(limit), min(abs(limit), float(value)))


def coupled_command(leader_linear, leader_angular, linear_sign, angular_sign,
                    follow_angular, max_linear, max_angular):
    """1호기 속도를 2호기 주행 명령으로 옮긴다.

    ``linear_sign`` 은 실측 보정값이다. 결합 기하(1호기가 앞인지 뒤인지, 2호기가
    180도 돌아 붙었는지)에 따라 같은 전진이 2호기에서는 후진일 수 있다.
    처음 결합하면 낮은 속도로 부호부터 확인할 것.

    ``follow_angular`` 는 **강체 결합이라 기본 켜짐이다** (2026-09-04 확인).

    강체로 붙으면 두 호기는 하나의 긴 물체다. 강체의 각속도는 어느 지점에서나
    같으므로, 2호기 궤도도 1호기와 **같은 각속도**로 명령해야 한다. 좌우를
    같은 속도로 굴리면(조향 0) 2호기는 회전을 막는 쪽으로 버텨서 1호기가 그
    저항까지 이겨야 한다 — 잠긴 캐스터와 같다.

    반대로 힌지로 붙는다면 2호기 방향은 조인트 각이 정하므로, 조인트 각을
    모르는 채로 조향을 복제하면 잭나이프가 난다. 그때는 꺼야 한다.

    ``angular_sign`` 은 ``linear_sign`` 과 같은 이유로 실측 대상이다 — 2호기가
    180도 돌아 붙으면 전진과 회전 **둘 다** 부호가 뒤집힌다.
    """
    if leader_linear is None:
        return 0.0, 0.0
    linear = clamp(float(linear_sign) * float(leader_linear), max_linear)
    if not follow_angular or leader_angular is None:
        return linear, 0.0
    return linear, clamp(float(angular_sign) * float(leader_angular), max_angular)


class CoupledDrive(Node):
    def __init__(self):
        super().__init__('coupled_drive')

        # 값은 mission/config/coupled.yaml 에서 조정한다.
        defaults = {
            'enabled': True,
            # 1호기 속도가 이 시간 끊기면 0을 낸다. 1호기가 50Hz 로
            # 보내므로 0.5 초면 25 프레임을 놓친 뒤다 — 확실한 두절이다.
            'leader_timeout_s': 0.5,
            # 결합이 locked 일 때만 굴린다. 결합 없이 시험하려면 false.
            'require_locked': True,
            # ★ 실측 전까지 부호를 믿지 말 것 (docstring 참고).
            'linear_sign': 1.0,
            # 2026-09-04 강체로 확인됨 -> 켠다. 근거는 docstring 참고.
            'follow_angular': True,
            'angular_sign': 1.0,
            'max_linear_mps': 0.35,
            'max_angular_rps': 0.30,
            # 1호기 발행(50Hz)보다 느리면 그 차이만큼 2호기 바퀴가 늦게
            # 따라간다. 같은 주기로 맞춘다.
            'publish_rate_hz': 50.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.enabled = bool(self._parameter('enabled'))
        self.leader_linear = None
        self.leader_angular = None
        self.leader_time = None
        self.coupling_phase = None
        self.reason = 'startup'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/coupled', 10)
        self.status_pub = self.create_publisher(
            String, 'mission/coupled/status', 10)
        # 호기 간 규약 토픽이라 절대 경로다.
        self.create_subscription(
            Twist, '/fleet/cmd_vel/unit2', self._on_leader_cmd, leader_qos())
        self.create_subscription(
            String, 'mission/coupling/status', self._on_coupling, 10)

        rate = max(1.0, float(self._parameter('publish_rate_hz')))
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f'coupled_drive ready | linear_sign={self._parameter("linear_sign")} '
            f'follow_angular={self._parameter("follow_angular")}')

    def _parameter(self, name):
        return self.get_parameter(name).value

    def _on_leader_cmd(self, msg):
        self.leader_linear = float(msg.linear.x)
        self.leader_angular = float(msg.angular.z)
        self.leader_time = time.monotonic()

    def _on_coupling(self, msg):
        try:
            self.coupling_phase = json.loads(msg.data).get('phase')
        except (TypeError, ValueError):
            pass

    def _leader_fresh(self, now):
        return (self.leader_time is not None and
                now - self.leader_time <= float(self._parameter('leader_timeout_s')))

    def _blocked(self, now):
        """굴리면 안 되는 이유. 없으면 None."""
        if not self.enabled:
            return 'disabled'
        if bool(self._parameter('require_locked')) and self.coupling_phase != 'locked':
            return 'not_locked'
        if self.leader_time is None:
            return 'leader_velocity_missing'
        if not self._leader_fresh(now):
            return 'leader_velocity_stale'
        return None

    def _tick(self):
        now = time.monotonic()
        self.reason = self._blocked(now) or ''
        cmd = Twist()
        if not self.reason:
            linear, angular = coupled_command(
                self.leader_linear, self.leader_angular,
                self._parameter('linear_sign'), self._parameter('angular_sign'),
                bool(self._parameter('follow_angular')),
                float(self._parameter('max_linear_mps')),
                float(self._parameter('max_angular_rps')))
            cmd.linear.x = linear
            cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        self.output = cmd

    def _publish_status(self):
        output = getattr(self, 'output', Twist())
        self.status_pub.publish(String(data=json.dumps({
            'active': not self.reason,
            'reason': self.reason,
            'coupling_phase': self.coupling_phase,
            'leader_linear': self.leader_linear,
            'leader_angular': self.leader_angular,
            'leader_fresh': self._leader_fresh(time.monotonic()),
            'linear': output.linear.x,
            'angular': output.angular.z,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = CoupledDrive()
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
