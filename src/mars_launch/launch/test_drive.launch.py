"""본체 전체 + 호기 로컬 시험 화면(:5000).

운용 UI(mars_console)는 사용자 PC에서 따로 띄운다. 이 화면은 그것과 별개로
**호기 위에서 도는 정비용 화면**이다 — 카메라, 상태, 비상 정지, 자동 결합.

    ros2 launch mars_launch test_drive.launch.py
    ros2 launch mars_launch test_drive.launch.py season:=escort
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context, *args, **kwargs):
    unit = int(LaunchConfiguration('unit').perform(context))
    launch_share = get_package_share_directory('mars_launch')
    drive_config = os.path.join(
        get_package_share_directory('drive'), 'config', 'arbiter.yaml')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_share, 'launch', 'robot.launch.py')),
            launch_arguments={
                'unit': str(unit),
                'season': LaunchConfiguration('season').perform(context),
            }.items()),
        Node(package='drive', executable='test_console', namespace=f'unit{unit}',
             output='screen', parameters=[drive_config, {'unit': unit}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('unit', default_value='2',
                              description='호기 번호 (2 또는 3)'),
        DeclareLaunchArgument('season', default_value='summer',
                              description='spring|summer|autumn|winter|escort'),
        OpaqueFunction(function=build),
    ])
