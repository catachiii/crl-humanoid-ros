import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    monitor_config = os.path.join(
        get_package_share_directory("crl_humanoid_monitor"),
        'config',
        'wf_tron1a_monitor.yaml'
    )

    hardware_config = os.path.join(
        get_package_share_directory("crl_humanoid_hardware"),
        'config',
        'tron1a_hardware.yaml'
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROBOT_TYPE', 'WF_TRON1A'),
        Node(
            package='crl_humanoid_hardware',
            namespace='tron1a_hardware',
            executable='hardware',
            parameters=[hardware_config],
            arguments=['--model', 'wf_tron1a'],
            output='screen'
        ),
        Node(
            package='crl_humanoid_monitor',
            namespace='tron1a_hardware',
            executable='monitor',
            parameters=[monitor_config],
            remappings=[
                ('monitor_joystick', 'remote')
            ]
        )
    ])
