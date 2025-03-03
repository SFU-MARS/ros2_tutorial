import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import subprocess
import time
import re
import xml.etree.ElementTree as ET

class MultiRobotSpawner(Node):
        def __init__(self):
                super().__init__('multi_robot_spawner')
                self.robots = []
                self.robot_count = 3
                self.config = {
                        'robot_colors': ['Blue', 'Red', 'Green'],
                        'robot_positions': [
                                [0.0, 0.0, 0.15],
                                [1.0, 0.0, 0.15],
                                [0.0, 1.0, 0.15]
                        ]
                }

                self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
                while not self.spawn_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info("waiting")
