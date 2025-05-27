from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    my_bot_path = get_package_share_directory('my_bot')
    thermal_path = get_package_share_directory('mlx90640_thermal')
    imu_params_path = os.path.join(get_package_share_directory('ros2_mpu6050'), 'config', 'params.yaml')

    urdf_path = PathJoinSubstitution([
        FindPackageShare("my_bot"),
        "description",
        "xacro",
        "rhex.urdf.xacro"
    ])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": Command(["xacro ", urdf_path])}],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='leg_odometry',
            executable='leg_odometry_node',
            name='leg_odometry_node',
            output='screen'
        ),



        # RPLIDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser'
            }]
        ),

        # MPU6050 IMU
        Node(
            package='ros2_mpu6050',
            executable='ros2_mpu6050',
            name='mpu6050',
            output='screen',
            parameters=[imu_params_path]
        ),

        # IMU Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[os.path.join(my_bot_path, 'config', 'madgwick_params.yaml')],
            remappings=[
                ('imu/data_raw', '/imu/mpu6050'),
                ('imu/data', '/imu/filtered')
            ]
        ),


        # Thermal Image Publisher Node
        Node(
            package='mlx90640_thermal',
            executable='publisher_node',
            name='thermal_publisher_node',
            output='screen'
        ),


        # Thermal Inference Node
        Node(
            package='mlx90640_thermal',
            executable='inference_node',
            name='thermal_inference_node',
            output='screen'
        ),

        # Thermal Marker Node
        Node(
            package='mlx90640_thermal',
            executable='thermal_marker_node',
            name='thermal_marker_node',
            output='screen'
        ),

        # Robot Localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(my_bot_path, 'config', 'ekf.yaml')],
            remappings=[
                ('/imu/data', '/imu/filtered')
            ]
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': os.path.join(my_bot_path, 'config', 'my_slam_params.yaml')
            }.items()

        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_bot_path, 'launch', 'custom_navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': os.path.join(my_bot_path, 'config', 'nav2_params.yaml'),
                'default_bt_xml_filename': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees',
                    'navigate_to_pose_w_replanning_and_recovery.xml'
                ),
                'map_subscribe_transient_local': 'true'
            }.items()
        ),

        Node(
            package='my_bot',
            executable='marker_goal_sender.py',
            name='marker_goal_sender',
            output='screen'
        ),



        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(my_bot_path, 'config', 'view_bot.rviz')]
        )
    ])