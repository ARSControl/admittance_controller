import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("admittance_controller"),
        "config",
        "joy_mode_command_params.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="admittance_controller",
                executable="joy_mode_command_node",
                name="joy_mode_command_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
