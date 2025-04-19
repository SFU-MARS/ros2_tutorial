from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_arg = DeclareLaunchArgument('config', default_value='robot_config_lab.yaml')
    map_arg = DeclareLaunchArgument('map', default_value='non_obstacle.yaml')
    
    config_file = LaunchConfiguration('config_file', default='robot_config_lab.yaml')
    
    robot_count = 2
    
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    map_path = os.path.join('/workspaces/ros2_tutorial/maps', 'non_obstcale_lab.yaml')

    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'localization_launch.py')),
        launch_arguments={'map': map_path}.items()
    )

    amcl_nodes = []
    for i in range(robot_count):
        amcl_nodes.append(
            Node(
                package='nav2_amcl',
                executable='amcl',
                namespace=f'tb_{i}',
                name=f'amcl_{i}',
                parameters=[{
                    'robot_model_type': 'differential',
                    'global_frame_id': 'map',
                    'odom_frame_id': f'tb_{i}/odom',
                    'base_frame_id': f'tb_{i}/base_footprint',
                    'use_sim_time': False,
                    'autostart': True,
                    'alpha1': 0.2,
                    'alpha2': 0.2,
                    'alpha3': 0.2,
                    'alpha4': 0.2,
                    'alpha5': 0.2,
                    'laser_max_beams': 180,
                    'laser_model_type': 'likelihood_field',
                }],
                remappings=[
                    ('scan', f'/tb_{i}/scan'),
                    ('map', '/map'),
                    ('initialpose', f'/tb_{i}/initialpose')
                ]
            )
        )
    
    
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

    initial_pose_event = RegisterEventHandler(
        OnProcessStart(
            target_action=global_position_provider,
            on_start=[
                LogInfo(msg="Global position provider started, launching initial pose publisher..."),
                initial_pose_publisher
            ]
        )
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

    bvc_controller_event = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_pose_publisher,
            on_exit=[
                LogInfo(msg="Initial pose publisher completed, launching BVC controller..."),
                bvc_controller
            ]
        )
    )
    
    # Create launch description
    ld = LaunchDescription([
        config_arg,
        map_arg,
        nav2_localization_launch,
        global_position_provider,
        initial_pose_event,
        bvc_controller_event
    ])

    for node in amcl_nodes:
        ld.add_action(node)
    
    
    return ld
