"""마커 pose와 UWB 거리로 1호기를 추종한다."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String


class FollowLeader(Node):
    def __init__(self):
        super().__init__('follow_leader')

        # 인수인계 수정 경계:
        # - 목표 거리·제어 이득·허용오차·최대 속도는 follow.yaml에서 조정한다.
        # - 센서 선택, fallback 조건, 거리/조향 계산식을 바꿀 때만 코드를 고친다.
        # 마커나 UWB가 끊기면 속도를 즉시 0으로 만드는 안전 동작은 유지한다.
        self.declare_parameter('target_distance_mm', 600.0)
        self.declare_parameter('k_linear', 0.0005)
        self.declare_parameter('k_angular', 1.0)
        self.declare_parameter('distance_tolerance_mm', 50.0)
        self.declare_parameter('lateral_tolerance_m', 0.03)
        # 마커 yaw(정면 정렬) 비례항 — 2026-08-29 추가. 리더가 제자리에서
        # 돌 때 팔로워도 같이 정렬하게 한다. k_angular(좌우 오프셋)는 이미
        # 실측 튜닝된 값이라 그대로 두고, 이 항만 더한다(angular.z에 합산
        # — motor.py가 linear.x는 공통 전진력, angular.z는 좌우 차동으로
        # 나누므로 Twist에 더하는 것 자체가 양쪽 모터 파워에 반영된다).
        # 기본 0.0(비활성) — 부호/크기 미실측. 실측 전엔 이 프로젝트의
        # 다른 미확정 값(contact_current_threshold_a 등)과 같은 원칙으로
        # 안전 측(꺼짐)에 둔다. TODO.md 11번 참고.
        self.declare_parameter('k_yaw', 0.0)
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('max_linear_mps', 0.30)
        self.declare_parameter('max_angular_rps', 0.50)
        self.declare_parameter('sensor_timeout_s', 0.5)
        self.declare_parameter('allow_marker_distance_fallback', False)
        # marker/pose의 z(거리)는 solvePnP 특성상 멀수록 잡음이 커진다
        # (60cm서 약 7%, base/marker_vision.md 참고) — UWB 폴백으로 쓸 때만
        # 이 계수로 지수이동평균해 속도 명령 떨림을 줄인다.
        self.declare_parameter('marker_distance_ema_alpha', 0.3)
        self.action = 'idle'
        self.pose = None
        self.pose_time = None
        self.yaw_rad = 0.0
        self.marker_distance_ema_mm = None
        self.uwb_mm = None
        self.uwb_time = None
        self.pub = self.create_publisher(Twist, 'cmd_vel/follow', 10)
        self.status_pub = self.create_publisher(String, 'mission/follow/status', 10)
        self.create_subscription(String, 'mission/action', self._on_action, 10)
        self.create_subscription(PoseStamped, 'marker/pose', self._on_pose, 10)
        self.create_subscription(Float32, 'uwb/distance_mm', self._on_uwb, 10)
        self.create_timer(0.05, self._tick)

    def _on_action(self, msg):
        try: self.action = str(json.loads(msg.data).get('action', 'idle'))
        except Exception: self.action = 'idle'

    def _on_pose(self, msg):
        self.pose, self.pose_time = msg, time.monotonic()
        self.yaw_rad = self._heading_from_quaternion(msg.pose.orientation)
        alpha = max(0.0, min(1.0, float(
            self.get_parameter('marker_distance_ema_alpha').value)))
        z_mm = float(msg.pose.position.z) * 1000.0
        self.marker_distance_ema_mm = (
            z_mm if self.marker_distance_ema_mm is None else
            (1.0 - alpha) * self.marker_distance_ema_mm + alpha * z_mm)

    @staticmethod
    def _heading_from_quaternion(orientation):
        """PoseStamped.orientation(쿼터니언)에서 로봇의 실제 좌우 회전
        (월드 기준 yaw, rad)을 뽑는다.

        2026-08-30 실측으로 발견/정정: aruco_tracker.py가 계산하는
        roll_deg/pitch_deg/yaw_deg는 마커 "자기 자신"의 로컬 축(X=마커
        가로변, Y=마커 세로변, Z=마커 면에서 튀어나오는 축) 기준
        Tait-Bryan 각도일 뿐, 마커가 로봇에 어떻게 붙어있는지는 전혀
        모른다. 이 로봇은 마커판이 뒤쪽에 수직으로(마커 로컬 Y축 = 세상의
        위쪽) 붙어있어서, 로봇이 제자리에서 도는 실제 회전(월드 yaw)은
        마커의 **로컬 Y축 회전**으로 나타난다 — aruco_tracker.py 기준으로는
        `yaw_deg`(로컬 Z축, 이미지 평면 안 회전 — 로봇 기준으로는 오히려
        롤/뱅킹에 해당)가 아니라 **`pitch_deg`**다. 실측 검증: 로봇을
        약 30도 돌렸을 때 `pitch_deg`는 -27.7도 변했지만(≈실제 회전과
        거의 일치) `yaw_deg`는 -0.26도만 변했다(로봇이 제자리 회전만
        했으니 실제 롤은 거의 없는 게 정상 — 즉 `yaw_deg`는 이 회전에
        거의 반응하지 않았다).

        marker_vision.py `_publish_pose`가 만드는 쿼터니언은 표준
        ZYX(요-피치-롤) 순서라, 여기서는 그 쿼터니언에서 표준 pitch(Y축)
        성분만 뽑는다(roll/yaw는 이 노드가 안 써서 생략). 이름은
        `_yaw_from_quaternion`이었다가 실제로 뽑는 게 쿼터니언의 "yaw"
        성분이 아니라 "pitch" 성분이라 헷갈리지 않게 `_heading_from_
        quaternion`으로 바꿨다 — 반환값의 의미(로봇의 실제 좌우 회전)는
        그대로다.
        """
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
        sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        return math.asin(sin_pitch)

    def _on_uwb(self, msg):
        self.uwb_mm, self.uwb_time = float(msg.data), time.monotonic()

    def _combine_angular(self, lateral, yaw_rad):
        """조향(angular.z) = 좌우 오프셋항 + yaw(정면 정렬)항의 합.

        두 항 다 "화면 중심 기준 오차"를 각자 비례 게인으로 줄이려는
        항이라 그냥 더한다 — motor.py가 이 하나의 angular.z 값을 좌우
        차동으로 나눠 양쪽 모터 파워에 반영하므로, 여기서 더하는 것
        자체가 "통합값을 양쪽 모터에 적용"하는 것과 같다.
        """
        angular_value = 0.0
        if abs(lateral) >= float(self.get_parameter('lateral_tolerance_m').value):
            angular_value += -float(self.get_parameter('k_angular').value) * lateral
        yaw_tolerance = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value))
        if abs(yaw_rad) >= yaw_tolerance:
            angular_value += float(self.get_parameter('k_yaw').value) * yaw_rad
        limit = float(self.get_parameter('max_angular_rps').value)
        return max(-limit, min(limit, angular_value))

    def _tick(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('sensor_timeout_s').value)
        marker_ready = self.pose is not None and now - self.pose_time <= timeout
        uwb_ready = self.uwb_mm is not None and now - self.uwb_time <= timeout
        fallback = bool(self.get_parameter('allow_marker_distance_fallback').value)
        cmd = Twist()
        phase = 'idle'
        reason = ''
        if self.action == 'follow':
            phase = 'following'
            if not marker_ready:
                reason = 'marker_timeout'
                self.marker_distance_ema_mm = None
            elif not uwb_ready and not fallback:
                reason = 'uwb_timeout'
            else:
                distance_mm = (self.uwb_mm if uwb_ready
                               else self.marker_distance_ema_mm)
                error = distance_mm - float(
                    self.get_parameter('target_distance_mm').value)
                tolerance = float(
                    self.get_parameter('distance_tolerance_mm').value)
                if error > tolerance:
                    cmd.linear.x = min(
                        float(self.get_parameter('max_linear_mps').value),
                        float(self.get_parameter('k_linear').value) * error)
                lateral = float(self.pose.pose.position.x)
                cmd.angular.z = self._combine_angular(lateral, self.yaw_rad)
        self.pub.publish(cmd)
        status = String()
        status.data = json.dumps({
            'phase': phase, 'active': self.action == 'follow',
            'marker_ready': marker_ready, 'uwb_ready': uwb_ready,
            'reason': reason, 'linear': cmd.linear.x, 'angular': cmd.angular.z,
            'yaw_deg': math.degrees(self.yaw_rad) if marker_ready else None,
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args); node = FollowLeader()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
