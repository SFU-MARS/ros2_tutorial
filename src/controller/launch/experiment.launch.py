import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("controller"), "param", "robot_params.yaml"
    )
    controller_node = Node(
        package="controller", executable="robot_controller", parameters=[config]
    )

    sim_node = Node(
        package="vicon_simulator",
        executable="robot_pose_simulator",
    )

    ld.add_action(controller_node)
    ld.add_action(sim_node)

    return ld
