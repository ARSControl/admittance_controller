import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("admittance_controller"),
        "config",
        "admittance_gui_params.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="admittance_controller",
                executable="admittance_gui_node",
                name="admittance_gui_node",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
