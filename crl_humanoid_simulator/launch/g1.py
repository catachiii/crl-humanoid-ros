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

    sim_config = os.path.join(
        get_package_share_directory("crl_humanoid_simulator"),
        'config',
        'g1_sim.yaml'
    )

    return LaunchDescription([
        Node(
            package='crl_humanoid_simulator',
            namespace='g1_sim',
            executable='sim',
            parameters=[sim_config],
            arguments=['--model', 'g1']
        ),
        Node(
            package='crl_humanoid_monitor',
            namespace='g1_sim',
            executable='monitor',
            parameters=[
                monitor_config,
                {
                    'model': 'g1',
                    'robot_xml_file': 'g1_description/scene_crl.xml'
                }
            ],
            remappings=[
                ('monitor_joystick', 'remote')
            ]
        )
    ])
