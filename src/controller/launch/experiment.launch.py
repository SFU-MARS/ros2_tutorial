from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_node = Node(
        package="controller",
        executable="robot_controller",
    )
    sim_node = Node(
        package="vicon_simulator",
        executable="robot_pose_simulator",
    )

    ld.add_action(controller_node)
    ld.add_action(sim_node)

    return ld



