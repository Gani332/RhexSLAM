from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Set ROS_PACKAGE_PATH so xacro can resolve package:// URIs
    set_ros_package_path = SetEnvironmentVariable(
        name="ROS_PACKAGE_PATH",
        value=EnvironmentVariable("AMENT_PREFIX_PATH")
    )

    # URDF from xacro (use source space!)
    xacro_file = PathJoinSubstitution([
        FindPackageShare("rhex_description"),
        "urdf",
        "rhex.xacro"
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        "--inorder ",  # optional but recommended
        xacro_file,
        " ",
        "use_sim:=false"
    ])

    robot_description = {"robot_description": robot_description_content}

    # Controller config
    controller_config = PathJoinSubstitution([
        FindPackageShare("rhex_control"),
        "config",
        "rhex_controllers.yaml"
    ])

    # robot_state_publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    # ros2_control_node (controller manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen"
    )

    # Delay starting controller manager to allow URDF to publish
    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    # Spawner nodes for controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen"
    )

    # Start spawners in sequence
    delayed_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    delayed_velocity = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[velocity_controller_spawner]
        )
    )

    return LaunchDescription([
        set_ros_package_path,
        robot_state_publisher,
        delayed_controller_manager,
        delayed_jsb,
        delayed_velocity,
    ])
