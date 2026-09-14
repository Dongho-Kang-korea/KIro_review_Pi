"""1호기·UI 없이 이 호기만으로 수동 주행한다. 정비와 하드웨어 시험용이다.

올리는 것은 **모터와 중재기 둘뿐**이다:

  - fleet follower 를 안 띄운다 → ``safety/stop`` 이 걸릴 일이 없다
    (follower 는 종료할 때 stop=true 를 남기므로 단독 주행에선 방해가 된다)
  - mission 노드를 안 띄운다 → UWB·IMU·전류 기준 미장착 게이트를 안 탄다
  - arbiter 를 ``start_mode: manual`` 로 열어 수동 후보를 최종 속도로 만든다

안전장치는 그대로다. 비상 정지가 최우선이고, ``cmd_vel/manual`` 이
``source_timeout_s`` (0.5초) 안에 갱신되지 않으면 arbiter 가 0 을 낸다.
모터 노드도 ``cmd_vel_timeout_s`` (0.5초) 로 자체 정지한다.

실행
----
조작 노드는 터미널(TTY)이 필요해 이 launch 에 넣지 않았다. 창 두 개를 쓴다::

    # 1번 창
    ros2 launch mars_launch manual.launch.py

    # 2번 창 — 도메인과 네임스페이스를 맞춰야 한다
    export ROS_DOMAIN_ID=10
    ros2 run drive manual --ros-args -r __ns:=/unit2

카메라 화면과 비상 정지 버튼이 필요하면 (:5000)::

    ros2 launch mars_launch manual.launch.py console:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def cfg(package, name):
    return os.path.join(get_package_share_directory(package), 'config', name)


def build(context, *args, **kwargs):
    unit = int(LaunchConfiguration('unit').perform(context))
    if unit != 2:      # 3호기 제외 (TODO 19)
        raise RuntimeError(f'unit 은 2 또는 3 이어야 한다: {unit}')
    namespace = f'unit{unit}'
    console = LaunchConfiguration('console')
    camera_env = {
        'PYTHONPATH': '/usr/local/lib/python3/dist-packages:' +
                      os.environ.get('PYTHONPATH', '')}

    return [
        Node(package='base', executable='motor', namespace=namespace,
             output='screen', parameters=[cfg('base', 'motor.yaml')]),

        # 실전 yaml 을 그대로 읽고 시작 모드만 덮어쓴다. 뒤 dict 가 이긴다.
        Node(package='drive', executable='arbiter', namespace=namespace,
             output='screen',
             parameters=[cfg('drive', 'arbiter.yaml'), {'start_mode': 'manual'}]),

        Node(package='base', executable='camera', namespace=namespace,
             output='screen', parameters=[cfg('base', 'camera.yaml')],
             additional_env=camera_env, condition=IfCondition(console)),
        Node(package='drive', executable='test_console', namespace=namespace,
             output='screen',
             parameters=[cfg('drive', 'arbiter.yaml'), {'unit': unit}],
             condition=IfCondition(console)),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),
        DeclareLaunchArgument('unit', default_value='2',
                              description='호기 번호 (2 또는 3)'),
        DeclareLaunchArgument(
            'console', default_value='false',
            description='카메라 + 시험 화면(:5000, 비상 정지)을 함께 띄운다'),
        OpaqueFunction(function=build),
    ])
