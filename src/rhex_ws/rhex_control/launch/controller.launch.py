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
        urdf_path,
    ])
    robot_description = {"robot_description": robot_description_content}

    # Path to controller config (not strictly needed here, already loaded by gazebo_ros2_control)
    controller_config = PathJoinSubstitution([
        FindPackageShare("rhex_control"),
        "config",
        "rhex_controllers.yaml"
    ])


    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    controller_manager= Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,controller_config],
        output="screen"  
    )

    jsb_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
    )

    velocity_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["velocity_controller"],
    )    

    return LaunchDescription([
        controller_manager,
        robot_state_publisher,
        jsb_spawner,
        velocity_spawner,
    ])
