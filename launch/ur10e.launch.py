import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_admittance = os.path.join(
        get_package_share_directory('admittance_controller'),
        'config',
        'ur10e_adm_params.yaml'
    )

    return LaunchDescription([
        Node(
            package = 'admittance_controller',
            name = 'admittance_control_node',
            executable = 'admittance_control_node',
            parameters = [config_admittance],
            remappings = [],
            output = 'screen'
        ),

    ])