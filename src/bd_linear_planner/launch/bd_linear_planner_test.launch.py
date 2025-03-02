import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bd_linear_planner_dir = get_package_share_directory('bd_linear_planner')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Include the Nav2 launch file with the linear planner
    planner_params = os.path.join(bd_linear_planner_dir, 'params', 'bd_linear_planner_params.yaml')
    
    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            planner_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the commands to the launch description
    ld.add_action(planner_cmd)

    return ld 