import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_admittance = os.path.join(
        get_package_share_directory('admittance_controller'),
        'config',
        'ur5e_adm_params.yaml'
    )

    config_energy_tank = os.path.join(
        get_package_share_directory('energy_tank'),
        'config',
        'energy_tank_params.yaml'
    )

    config_perception = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'perception_params.yaml'
    )

    config_varadm_optimizer = os.path.join(
        get_package_share_directory('variable_admittance_optimizer'),
        'config',
        'variable_admittance_optimizer_params.yaml'
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

        # Node(
        #     package = 'energy_tank',
        #     name = 'energy_tank_node',
        #     executable = 'energy_tank_node',
        #     parameters = [config_energy_tank],
        #     remappings = [],
        #     output = 'screen'
        # ),

        # Node(
        #     package = 'perception',
        #     name = 'perception_node',
        #     executable = 'perception_node',
        #     parameters = [config_perception],
        #     remappings = [],
        #     output = 'screen'
        # ),

        # Node(
        #     package = 'variable_admittance_optimizer',
        #     name = 'variable_admittance_optimizer_node',
        #     executable = 'variable_admittance_optimizer_node',
        #     parameters = [config_varadm_optimizer],
        #     remappings = [],
        #     output = 'screen'
        # ),

    ])