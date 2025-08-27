import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("crl_humanoid_simulator"),
        'config',
        'g1_sim.yaml'
    )

    return LaunchDescription([
        Node(
            package='crl_humanoid_simulator',
            namespace='g1_sim',
            executable='sim',
            parameters=[config],
        ),
        Node(
            package='crl_humanoid_monitor',
            namespace='g1_sim',
            executable='monitor',
            remappings=[
                ('monitor_joystick', 'remote')
            ]
        )
    ])
