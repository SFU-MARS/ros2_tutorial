from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_arg = DeclareLaunchArgument('config', default_value='robot_config_lab.yaml')
    map_arg = DeclareLaunchArgument('map', default_value='non_obstacle.yaml')
    
    config_file = LaunchConfiguration('config_file', default='robot_config_lab.yaml')
    
    robot_count = 2
    
    
    # Launch global position provider
    global_position_provider = Node(
        package='multiple_robots_simulation',
        executable='global_position_provider',
        name='global_position_provider'
    )
    
    initial_pose_publisher = Node(
        package='multiple_robots_simulation',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        parameters=[{
            'config_file': config_file
        }]
    )
    
    # BVC controller with config filename
    bvc_controller = Node(
        package='multiple_robots_simulation',
        executable='bvc_controller',
        name='bvc_controller',
        parameters=[{
            'safety_radius': 0.3,
            'update_rate': 5,
            'max_linear_speed': 0.2,
            'goal_tolerance': 0.2,
            'config_file': config_file,
            'world_size': 15.0,
            'max_angular_speed': 0.5,
            'angle_tolerance': 0.1
        }]
    )
    
    # Create launch description
    ld = LaunchDescription([
        config_arg,
        map_arg,
        bvc_controller
    ])
    
    
    return ld
