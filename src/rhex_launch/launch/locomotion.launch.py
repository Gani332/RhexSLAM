from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    giga_hw_pkg = get_package_share_directory('giga_hw_interface')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(giga_hw_pkg, 'launch', 'giga.launch.py')
            )
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rhex_control', 'rhex_gait.py'],
            output='screen'
        )
    ])
