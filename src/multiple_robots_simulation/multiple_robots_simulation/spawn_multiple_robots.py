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
                self.robot_count = 1
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

                self.spawn_robots()

        def spawn_robots(self):
                package_dir = get_package_share_directory('multiple_robots_simulation')
                urdf_path = os.path.join(package_dir, 'models', 'box_bot.urdf')
                
                for i in range(self.robot_count):
                        robot_name = f'robot_{i}'
                        robot_pos = self.config['robot_positions'][i]
                        
                        urdf_content = ET.tostring(ET.parse(urdf_path).getroot(), encoding='unicode')  
                        
                        # Create pose for the robot
                        pose = Pose()
                        pose.position.x = float(robot_pos[0])
                        pose.position.y = float(robot_pos[1])
                        pose.position.z = float(robot_pos[2])
                        
                        # Request to spawn robot
                        request = SpawnEntity.Request()
                        request.name = robot_name
                        request.xml = urdf_content
                        request.robot_namespace = robot_name
                        request.initial_pose = pose
                        request.reference_frame = 'world'
                        
                        # Send request
                        future = self.spawn_client.call_async(request)
                        rclpy.spin_until_future_complete(self, future)
                        
                        if future.result() is not None:
                                self.get_logger().info(f'Spawned {robot_name} successfully')
                                self.robots.append(robot_name)
                        else:
                                self.get_logger().error(f'Failed to spawn {robot_name}')
                                
                        time.sleep(0.5)


def main(args=None):
        rclpy.init(args=args)
        node = MultiRobotSpawner()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()