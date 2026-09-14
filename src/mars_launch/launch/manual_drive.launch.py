"""MARS 2호기용 Domain 77 격리 수동 주행 시험 구성."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    motor_config = os.path.join(
        get_package_share_directory('base'), 'config', 'motor.yaml')
    start_manual = LaunchConfiguration('start_manual')
    start_motor = LaunchConfiguration('start_motor')
    test_domain = LaunchConfiguration('test_domain')
    allow_unconfirmed_arm = LaunchConfiguration('allow_unconfirmed_arm')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('start_manual', default_value='true'),
        DeclareLaunchArgument('start_motor', default_value='true'),
        DeclareLaunchArgument('test_domain', default_value='77'),
        DeclareLaunchArgument('namespace', default_value='unit2'),
        DeclareLaunchArgument('allow_unconfirmed_arm', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', test_domain),
        Node(
            package='drive', executable='manual', output='screen',
            namespace=namespace, emulate_tty=True,
            parameters=[os.path.join(
                get_package_share_directory('drive'), 'config', 'arbiter.yaml')],
            condition=IfCondition(start_manual)),
        Node(
            package='drive', executable='arbiter', namespace=namespace,
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('drive'), 'config', 'arbiter.yaml'),
                {'start_mode': 'manual'}]),
        Node(
            package='base', executable='motor', namespace=namespace,
            output='screen',
            condition=IfCondition(start_motor),
            parameters=[motor_config, {
                'arm_on_start': False,
                'allow_unconfirmed_arm': ParameterValue(
                    allow_unconfirmed_arm, value_type=bool),
            }]),
    ])
