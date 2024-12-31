from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Path to the URDF file
    urdf_file_path = "/home/helium/fyp_ws/src/medibot/medibot_simulations/medibot_gazebo/urdf/medibot_medibotv4.urdf"  # Update with your URDF file path

    # Use xacro to process the URDF file
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file_path]),
        value_type=str,
    )

    return LaunchDescription([
        # Joint State Publisher
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),

        # Joint State Publisher GUI
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", "/home/helium/fyp_ws/src/medibot/medibot_simulations/medibot_gazebo/rviz/urdf.rviz"],  # Update with your RViz config path if available
            output="screen",
        ),
    ])
