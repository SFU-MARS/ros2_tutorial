import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('multiple_robots_simulation')
    

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_dir, 'launch', 'start_empty_world.launch.py')])
    )
    
    
    return LaunchDescription([
        gazebo_launch,
    ])
