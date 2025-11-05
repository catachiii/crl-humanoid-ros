import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    monitor_config = os.path.join(
        get_package_share_directory("crl_humanoid_monitor"),
        'config',
        'wf_tron1a_monitor.yaml'
    )

    sim_config = os.path.join(
        get_package_share_directory("crl_humanoid_simulator"),
        'config',
        'wf_tron1a_sim.yaml'
    )

    return LaunchDescription([
        Node(
            package='crl_humanoid_simulator',
            namespace='wf_tron1a_sim',
            executable='sim',
            parameters=[sim_config],
            arguments=['--model', 'wf_tron1a']
        ),
        Node(
            package='crl_humanoid_monitor',
            namespace='wf_tron1a_sim',
            executable='monitor',
            parameters=[
                monitor_config,
                {'model': 'wf_tron1a',
                'robot_xml_file': 'wf_tron1a_description/xml/scene_crl.xml'}
            ],
            remappings=[('monitor_joystick', 'remote')]
        )
    ])
