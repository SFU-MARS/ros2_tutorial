#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    
    robot_spawner = Node(
        package='multiple_robots_simulation',
        executable='spawn_multiple_robots',
        name='multi_robot_spawner',
        output='screen'
    )
    
    return LaunchDescription([
        robot_spawner
    ])
