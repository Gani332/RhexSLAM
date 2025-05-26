from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to your URDF file (xacro)
    urdf_path = PathJoinSubstitution([
        FindPackageShare("rhex_description"),
        "urdf",
        "rhex.xacro"
    ])

    # Process xacro file
    robot_description_content = Command([
        "xacro ",
        urdf_path
    ])
    robot_description = {"robot_description": robot_description_content}

    # Path to controller config (not strictly needed here, already loaded by gazebo_ros2_control)
    controller_config = PathJoinSubstitution([
        FindPackageShare("rhex_control"),
        "config",
        "rhex_controllers.yaml"
    ])

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller"],
        ),

    ])
