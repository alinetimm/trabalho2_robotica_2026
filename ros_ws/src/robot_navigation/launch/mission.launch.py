import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_yaml = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'mission.yaml'
    )

    navigator_node = Node(
        package='robot_navigation',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[mission_yaml],
    )

    return LaunchDescription([navigator_node])
