from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_arg = DeclareLaunchArgument('config', default_value='robot_config_lab_04_21.yaml')
    map_arg = DeclareLaunchArgument('map', default_value='non_obstacle_lab_04_21.yaml')
    
    robot_count = 2

    config_file = LaunchConfiguration('config', default='robot_config_lab_04_21.yaml')
    map_path = os.path.join('/workspaces/ros2_tutorial/maps', 'non_obstacle_lab_04_21.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_path,
            'use_sim_time': False,
            'autostart': True,
            'frame_id': 'map'
        }]
    )
    
    amcl_nodes = []
    for i in range(robot_count):
        amcl_nodes.append( Node(
            package='nav2_amcl',
            executable='amcl',
            namespace=f'tb_{i}',
            name=f'amcl_{i}',
            output='screen',
            parameters=[{
                'publish_tf': True,
                'bond_timeout': 60.0,
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'global_frame_id': 'map',
                'tf_broadcast': True, 
                'tf_broadcast_rate':20,
                'odom_frame_id': f'tb_{i}/odom',
                'base_frame_id': f'tb_{i}/base_footprint',
                'use_sim_time': True,
                'autostart': True,
                'transform_tolerance': 0.5, 
                'publish_tf_always': True,   
                'alpha1': 0.2, 
                'alpha2': 0.2,  
                'alpha3': 0.2,  
                'alpha4': 0.2,  
                'tf_delay': 0.1,
                'resample_interval': 1,
                'min_particles': 100,     
                'max_particles': 5000,
                'recovery_alpha_slow': 0.001,
                'recovery_alpha_fast': 0.1,
                'initial_pose_x_stddev': 0.05,
                'initial_pose_y_stddev': 0.05,
                'initial_pose_a_stddev': 0.05,
                # Improved laser parameters
                'laser_max_beams': 180,
                'laser_model_type': 'likelihood_field',
                # Force publications
                'set_initial_pose': True,
                'first_map_only': False,
                'debug_level' : 3,
                'always_reset_initial_pose': True,
                # Recovery parameters
                'recovery_alpha_slow': 0.001,
                'recovery_alpha_fast': 0.1,
                'update_min_d': 0.1,          
                'update_min_a': 0.1,          
                'resample_interval': 1,       # Update more frequently
                'transform_tolerance': 0.5,   # Reasonable tolerance for transforms
                'recovery_alpha_slow': 0.001, # Improve recovery behavior
                'recovery_alpha_fast': 0.1,
                'laser_max_beams': 30,       
                'tf_message_filter_queue_size': 20
            }],
            remappings=[
                ('scan', f'/tb_{i}/scan'),
                ('map', '/map'),
                ('initialpose', f'/tb_{i}/initialpose')
            ]
        )
    )
    
        
    lifecycle_node_names = ['/map_server']
    for i in range(robot_count):
            lifecycle_node_names.append(f'/tb_{i}/amcl_{i}')

    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                    'autostart': True,
                    'node_names': lifecycle_node_names,
                    'bond_timeout': 0.0,
                    'bond_required': False,
                    'attempt_respawn_reconnection': False
            }]
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
    
    # BVC controller with config filename
    bvc_controller = Node(
            package='multiple_robots_simulation',
            executable='bvc_controller',
            name='bvc_controller',
            parameters=[{
                    'safety_radius': 0.3,
                    'update_rate': 5,
                    'max_linear_speed': 0.2,
                    'goal_tolerance': 0.01,
                    'config_file': config_file,
                    'world_size': 15.0,
                    'max_angular_speed': 0.5,
                    'angle_tolerance': 0.1
            }]
    )
    
    # Wait for lifecycle manager to start, then launch the initial pose publisher after a delay
    lifecycle_start_event = RegisterEventHandler(
            OnProcessStart(
                    target_action=lifecycle_manager,
                    on_start=[
                            LogInfo(msg="Lifecycle manager started, waiting for nodes to activate..."),
                            TimerAction(
                                    period=5.0,  # 5-second delay to allow nodes to activate
                                    actions=[
                                            LogInfo(msg="Launching initial pose publisher..."),
                                            initial_pose_publisher
                                    ]
                            )
                    ]
            )
    )
    
    # Wait for initial pose publisher to exit, then launch the BVC controller
    bvc_controller_event = RegisterEventHandler(
            OnProcessExit(
                    target_action=initial_pose_publisher,
                    on_exit=[
                            LogInfo(msg="Initial pose publisher completed, launching BVC controller..."),
                            bvc_controller
                    ]
            )
    )
    
    sime_time = LaunchConfiguration('use_sim_time', default= 'true')
    
    odometry_node_0 = Node(
        package='tf2_ros',
        namespace= 'scan_to_map',
        executable='static_transform_publisher',
        name='initial_map_odom_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'tb_0/odom']
    )
    
    odometry_node_1 = Node(
        package='tf2_ros',
        namespace= 'scan_to_map',
        executable='static_transform_publisher',
        name='initial_map_odom_transform',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'tb_1/odom']
    )
    
    time_synced_transform_publisher = Node(
            package='multiple_robots_simulation',
            executable='time_synced_transform_publisher',
            name='time_synced_transform_publisher',
            parameters=[{
                'use_sim_time': True  
            }]
    )
    
    
    ld = LaunchDescription([
            config_arg,
            map_arg
    ])
    
    ld.add_action(time_synced_transform_publisher)
    
            
    ld.add_action(odometry_node_0) 
    ld.add_action(odometry_node_1) 
    ld.add_action(map_server_node)
    ld.add_action(amcl_nodes[0])
    ld.add_action(amcl_nodes[1])
    ld.add_action(lifecycle_manager)
    ld.add_action(global_position_provider)
    ld.add_action(lifecycle_start_event)
    ld.add_action(bvc_controller_event)
    
    return ld