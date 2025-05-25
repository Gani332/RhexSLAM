from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mlx90640_thermal',
            executable='publisher_node',
            name='thermal_publisher',
            output='screen'
        ),
        Node(
            package='mlx90640_thermal',
            executable='inference_node',
            name='thermal_inference',
            output='screen'
        ),

        Node(
            package='mlx90640_thermal',
            executable='thermal_marker_node',
            name='thermal_marker_node',
            output='screen'
        )

    ])
