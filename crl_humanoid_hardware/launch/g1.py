import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    monitor_config = os.path.join(
        get_package_share_directory("crl_humanoid_monitor"),
        'config',
        'g1_monitor.yaml'
    )

    hardware_config = os.path.join(
        get_package_share_directory("crl_humanoid_hardware"),
        'config',
        'g1_hardware.yaml'
    )

    return LaunchDescription([
        Node(
            package='crl_humanoid_hardware',
            namespace='g1_hardware',
            executable='hardware',
            parameters=[hardware_config],
            arguments=['--model', 'g1'],
            output='screen'
        ),
        Node(
            package='crl_humanoid_monitor',
            namespace='g1_hardware',
            executable='monitor',
            parameters=[monitor_config],
            remappings=[
                ('monitor_joystick', 'remote')
            ]
        )
    ])
