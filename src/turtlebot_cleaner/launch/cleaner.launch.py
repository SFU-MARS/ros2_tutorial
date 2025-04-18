from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    cleaner_node = Node(
        package='turtlebot_cleaner',
        executable='cleaner_node',
        name='cleaner_node',
        output='screen',
        emulate_tty=True
    )
    
    ld = LaunchDescription()
    
    ld.add_action(cleaner_node)
    
    return ld 