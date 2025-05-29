from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare use_sim argument (default true for gazebo)
    use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Use simulation or real hardware"
    )

    use_sim = LaunchConfiguration("use_sim")

    # Path to the robot's xacro file
    urdf_path = PathJoinSubstitution([
        FindPackageShare("rhex_description"),
        "urdf",
        "rhex.xacro"
    ])

    # Generate robot_description from xacro with use_sim flag
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        urdf_path,
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen"
    )

    # Launch Gazebo environment
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ])
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "rhex",
            "-x", "0", "-y", "0", "-z", "0",
        ],
        output="screen"
    )

    return LaunchDescription([
        use_sim_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])
