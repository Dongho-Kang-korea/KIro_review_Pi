"""MARS 2호기 **단독 벤치 시험용** 런치 — 1호기 없이 주행·추종·컨베이어·
솔레노이드·ArUco 인식을 한 런치에서 모두 켜고 웹 콘솔로 확인한다.

기본값은 motor 포함이다. 웹 입력은 ``/unit2/cmd_vel/manual`` 에서
arbiter를 거쳐 ``/unit2/cmd_vel`` 과 motor/CAN으로 전달된다.

카메라(``start_camera``), 솔레노이드·컨베이어 수동 조작(항상 켜짐,
``base/solenoid``+``mission/cargo_load``), 추종 모드 시험용
``fleet/follower``+``mission/follow_leader``(``start_fleet``)도 같이 뜬다.
같은 ``start_fleet``에 ``fleet/rc_bridge``(RC 조종기 모드, 1호기가 채널
값을 넘겨야 실제로 움직임 — TODO.md 14번, 토픽/채널 매핑 임시값)도 묶여
있다.

**1호기 없이 추종을 시험하려면** (``start_fake_leader``, 기본 켜짐) —
follower는 원래 1호기의 ``/fleet/state``가 살아 있어야만 자동으로
추종 액션을 걸고 arbiter도 그게 있어야 ``mode_cmd=follow``를 받아준다.
2026-08-29부터 ``fleet/fake_leader``가 그 신호를 대신 흉내 내 웹 콘솔에서
바로 추종 모드를 켜볼 수 있게 했다 — 거리 제어는 ArUco 마커
pose(``allow_marker_distance_fallback: true``, UWB 미장착 상태에서도
동작)로 이뤄진다. **주의**: 실물 1호기와 같은 ``ROS_DOMAIN_ID``에서
같이 켜면 안 된다(``/fleet/state``가 서로 충돌) — 이 런치의 기본
``test_domain``(77)이 실물 운용 도메인(10)과 분리돼 있는 한 안전하다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config(package, name):
    return os.path.join(get_package_share_directory(package), 'config', name)


def generate_launch_description():
    test_domain = LaunchConfiguration('test_domain')
    start_motor = LaunchConfiguration('start_motor')
    start_imu = LaunchConfiguration('start_imu')
    start_camera = LaunchConfiguration('start_camera')
    start_fleet = LaunchConfiguration('start_fleet')
    start_fake_leader = LaunchConfiguration('start_fake_leader')
    fake_leader_section = LaunchConfiguration('fake_leader_section')

    # Picamera2(libcamera)는 이 PYTHONPATH가 있어야 import된다 — robot.launch.py와
    # 동일한 처리다.
    camera_env = {
        'PYTHONPATH': '/usr/local/lib/python3/dist-packages:' +
                      os.environ.get('PYTHONPATH', '')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_domain', default_value='77',
            description='격리된 웹 수동시험 ROS Domain'),
        DeclareLaunchArgument(
            'start_motor', default_value='true',
            description='기본 true: motor 노드와 CAN 초기화를 함께 실행'),
        DeclareLaunchArgument(
            'start_imu', default_value='true',
            description='기본 true: BNO086 IMU와 RPY 토픽을 함께 실행'),
        DeclareLaunchArgument(
            'start_camera', default_value='true',
            description='기본 true: camera+marker_vision을 함께 실행해 웹 화면에'
                        ' 영상을 띄운다'),
        DeclareLaunchArgument(
            'start_fleet', default_value='true',
            description='기본 true: fleet/follower+mission/follow_leader를 함께'
                        ' 실행해 추종 모드를 시험할 수 있게 한다(1호기 또는'
                        ' start_fake_leader가 /fleet/state를 발행해야 움직인다)'),
        DeclareLaunchArgument(
            'start_fake_leader', default_value='true',
            description='기본 true: 1호기 없이 벤치에서 추종을 시험할 수 있게'
                        ' fleet/fake_leader가 /fleet/state를 대신 발행한다.'
                        ' 실물 1호기와 같은 ROS_DOMAIN_ID에서는 반드시 false로'
                        ' 끌 것(신호 충돌).'),
        DeclareLaunchArgument(
            'fake_leader_section', default_value='spring',
            description='fake_leader가 흉내 낼 계절(summer면 개별 명령이'
                        ' 필요해 자동 추종이 안 걸린다 — summer 화물 적재'
                        ' 시나리오를 시험할 게 아니면 기본값을 쓴다)'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', test_domain),

        Node(
            package='drive', executable='arbiter', namespace='unit2',
            output='screen',
            parameters=[config('drive', 'arbiter.yaml'),
                        {'start_mode': 'manual'}]),

        Node(
            package='mars_console', executable='console', output='screen',
            parameters=[config('mars_console', 'console.yaml')]),

        Node(
            package='base', executable='imu', namespace='unit2',
            output='screen', condition=IfCondition(start_imu),
            parameters=[config('base', 'imu.yaml')]),

        Node(
            package='base', executable='motor', namespace='unit2',
            output='screen', condition=IfCondition(start_motor),
            parameters=[config('base', 'motor.yaml')]),

        Node(
            package='base', executable='camera', namespace='unit2',
            output='screen', condition=IfCondition(start_camera),
            additional_env=camera_env,
            parameters=[config('base', 'camera.yaml')]),

        Node(
            package='base', executable='marker_vision', namespace='unit2',
            output='screen', condition=IfCondition(start_camera),
            parameters=[config('base', 'marker.yaml')]),

        # 솔레노이드/컨베이어 수동 조작(웹 콘솔)이 실제로 뭔가를 구동하려면
        # 이 두 노드가 떠 있어야 한다 — robot.launch.py와 같이 게이트 없이
        # 항상 띄운다(계절/모드와 무관하게 안전하게 대기만 하는 노드들이다).
        Node(
            package='base', executable='solenoid', namespace='unit2',
            output='screen', parameters=[config('base', 'solenoid.yaml')]),

        Node(
            package='mission', executable='cargo_load', namespace='unit2',
            output='screen', parameters=[config('mission', 'cargo.yaml')]),

        Node(
            package='fleet', executable='follower', namespace='unit2',
            output='screen', condition=IfCondition(start_fleet),
            parameters=[config('fleet', 'follower.yaml'), {'unit': 2}]),

        # RC(무선 조종기) 채널 중계 — start_fleet에 묶어 뒀다(1호기 통신
        # 계열 노드). 벤치에서는 1호기가 없어 아무 채널 데이터도 안 오니
        # 켜져 있어도 harmless — 콘솔에서 "조종기" 모드 버튼만 미리 확인
        # 가능하다. 토픽/채널 매핑은 임시값 — TODO.md 14번.
        Node(
            package='fleet', executable='rc_bridge', namespace='unit2',
            output='screen', condition=IfCondition(start_fleet),
            parameters=[config('fleet', 'rc_bridge.yaml'), {'unit': 2}]),

        Node(
            package='mission', executable='follow_leader', namespace='unit2',
            output='screen', condition=IfCondition(start_fleet),
            parameters=[config('mission', 'follow.yaml')]),

        # 1호기 없이 벤치에서 추종을 시험하기 위한 가짜 /fleet/state 발행
        # (namespace 없음 -- fleet 프로토콜 토픽은 절대경로다). 실물
        # 1호기가 있으면 start_fake_leader:=false로 반드시 끈다.
        Node(
            package='fleet', executable='fake_leader',
            output='screen', condition=IfCondition(start_fake_leader),
            parameters=[{'section': fake_leader_section}]),
    ])
