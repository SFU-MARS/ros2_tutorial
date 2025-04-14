#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    config_file = LaunchConfiguration('config_file', default='robot_config.yaml')
    urdf_file = LaunchConfiguration('urdf_file', default='box_bot.urdf')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='robot_config.yaml',
    )

    declare_urdf_file = DeclareLaunchArgument(
        'urdf_file',
        default_value='box_bot.urdf',
    )
    
    
    robot_spawner = Node(
        package='multiple_robots_simulation',
        executable='spawn_multiple_robots',
        name='multi_robot_spawner',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'urdf_file': urdf_file
        }]
    )

    bvc_controller = Node(
        package='multiple_robots_simulation',
        executable='bvc_controller',
        name='bvc_controller',
        output='screen',
        parameters=[
            {'safety_radius': 0.3},
            {'update_rate': 2},
            {'max_linear_speed': 0.2},
            {'goal_tolerance': 0.2},
            {'config_file': config_file},
            {'world_size': 15.0},
            {"max_angular_speed": 0.5},
            {"angle_tolerance": 0.1}
        ]
    )



    
    return LaunchDescription([
        declare_config_file,
        declare_urdf_file,
        robot_spawner
    ])
