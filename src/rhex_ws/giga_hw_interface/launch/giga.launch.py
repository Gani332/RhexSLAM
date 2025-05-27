from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get robot description from xacro
    xacro_file = PathJoinSubstitution([
        FindPackageShare("rhex_description"),
        "urdf",
        "rhex.xacro"
    ])
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", xacro_file, " use_sim:=false"
    ])
    robot_description = {"rsp1/robot_description": robot_description_content}

    # Controller config file
    controller_config = PathJoinSubstitution([
        FindPackageShare("rhex_control"),
        "config",
        "rhex_controllers.yaml"
    ])

    # State publisher
    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[robot_description],
    #     output="screen"
    # )

    # Controller manager node (ROS 2 control node)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen"
    )

    # Delayed startup to ensure /robot_description is published
    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    # Spawner: joint_state_broadcaster FIRST
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    # Spawner: velocity_controller AFTER joint_state_broadcaster
    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        output="screen"
    )

    # Chain: joint_state_broadcaster starts after controller_manager
    delayed_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[jsb_spawner]
        )
    )

    # Chain: velocity_controller starts after jsb is running
    delayed_velocity = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=jsb_spawner,
            on_start=[velocity_spawner]
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        delayed_controller_manager,
        delayed_jsb,
        delayed_velocity,
    ])
