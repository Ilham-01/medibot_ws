
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file_path = '/home/helium/fyp_ws/src/medibot/medibot_simulations/medibot_gazebo/urdf/medibot_medibotv4.urdf'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file_path, 'r').read(),
                'use_sim_time': False
            }]
        )
    ])
