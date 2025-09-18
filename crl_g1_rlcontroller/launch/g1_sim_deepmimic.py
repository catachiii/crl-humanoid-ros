import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    monitor_config = os.path.join(
        get_package_share_directory("crl_g1_rlcontroller"),
        'config',
        'g1_monitor.yaml'
    )

    sim_config = os.path.join(
        get_package_share_directory("crl_g1_rlcontroller"),
        'config',
        'g1_simulator_deepmimic.yaml'
    )


    return LaunchDescription([
        Node(
            package='crl_g1_rlcontroller',
            namespace='g1_sim',
            executable='simulator_main_deepmimic',
            parameters=[sim_config],
            arguments=['--model', 'g1']
        ),
        Node(
            package='crl_g1_rlcontroller',
            namespace='g1_sim',
            executable='monitor_main_deepmimic',
            parameters=[monitor_config],
            remappings=[
                ('monitor_joystick', 'remote')
            ]
        )
    ])
