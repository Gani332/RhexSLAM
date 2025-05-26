import os
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    # Set ROS_PACKAGE_PATH so xacro can resolve package:// URIs
    set_ros_package_path = SetEnvironmentVariable(
        name="ROS_PACKAGE_PATH",
        value=os.environ.get("AMENT_PREFIX_PATH", "")
    )

    # Use absolute path to source URDF (avoid install path resolution)
    xacro_file = os.path.join(
        os.path.expanduser("~"),
        "dev_ws",
        "src",
        "rhex_description",
        "urdf",
        "rhex.xacro"
    )

    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_file,
        " ",
        "use_sim:=false"
    ])

    robot_description = {"robot_description": robot_description_content}

    # Controller config path
    controller_config = os.path.join(
        os.path.expanduser("~"),
        "dev_ws",
        "src",
        "rhex_control",
        "config",
        "rhex_controllers.yaml"
    )

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

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager]
    )

    # Spawner nodes
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
