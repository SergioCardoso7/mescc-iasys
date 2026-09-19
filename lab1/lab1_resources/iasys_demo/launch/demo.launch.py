import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('iasys_demo')
    rviz_config = os.path.join(package_dir, 'rviz', 'iasys_demo.rviz')
    controller_enabled = LaunchConfiguration('controller_enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_enabled',
            default_value='true',
            description='Start the simple controller node.'),
        Node(package='iasys_demo', executable='world_node', output='screen'),
        Node(package='iasys_demo', executable='vehicle_simulator', output='screen'),
        Node(package='iasys_demo', executable='simple_planner', output='screen'),
        Node(
            package='iasys_demo',
            executable='simple_controller',
            output='screen',
            condition=IfCondition(controller_enabled)),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),
    ])
