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

                self.spawn_robots()

        def spawn_robots(self):
                package_dir = get_package_share_directory('multiple_robots_simulation')
                urdf_path = os.path.join(package_dir, 'models', 'box_bot.urdf')
                
                for i in range(self.robot_count):
                        robot_name = f'tb_{i}'
                        robot_pos = self.config['robot_positions'][i]
                        
                        urdf_content = self.modify_urdf_namespace(urdf_path, robot_name) 
                        
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

        def modify_urdf_namespace(self, urdf_file_path, robot_namespace):
                # modify based on: https://bitbucket.org/theconstructcore/box_bot/src/foxy/box_bot_description/launch/spawn_box_bot_v2.py
                try:
                        tree = ET.parse(urdf_file_path)
                        root = tree.getroot()
                        diff_drive_plugin = None
                        imu_plugin = None
                
                        for plugin in root.findall('.//plugin'):
                                if plugin.get('name') == 'differential_drive_controller':
                                        diff_drive_plugin = plugin
                                elif plugin.get('name') == 'box_bot_imu_plugin':
                                        imu_plugin = plugin
                                
                        if diff_drive_plugin is not None:
                                ros_element = diff_drive_plugin.find('./ros')
                                ros_element = ET.SubElement(diff_drive_plugin, 'ros')
                                
                                # Add namespace element and tf remapping
                                namespace_element = ET.SubElement(ros_element, 'namespace')
                                namespace_element.text = '/' + robot_namespace
                                remap_element = ET.SubElement(ros_element, 'remapping')
                                remap_element.text = '/tf:=/' + robot_namespace + '/tf'
                                
                                # Update odometry frames
                                odometry_frame = diff_drive_plugin.find('./odometry_frame')
                                odometry_frame.text = robot_namespace + '/odom'
                                robot_base_frame = diff_drive_plugin.find('./robot_base_frame')
                                robot_base_frame.text = robot_namespace + '/chassis'
                                
                                # Update topic names
                                cmd_vel_topic = diff_drive_plugin.find('./cmd_vel_topic')
                                cmd_vel_topic.text = robot_namespace + '/cmd_vel'
                                
                                odometry_topic = diff_drive_plugin.find('./odometry_topic')
                                odometry_topic.text = robot_namespace + '/odom'
                        
                        if imu_plugin is not None:
                                ros_element = imu_plugin.find('./ros')
                                if ros_element is not None:
                                        for arg in ros_element.findall('./argument'):
                                                if 'out:=' in arg.text:
                                                        pass

                                        remap_element = ET.SubElement(ros_element, 'remapping')
                                        remap_element.text = '/tf:=/' + robot_namespace + '/tf'
                                
                        return ET.tostring(root, encoding='unicode')
                
                except Exception as e:
                        #Can't modify the URDF file 
                        self.get_logger().error(f'Error modifying URDF: {e}')
                        with open(urdf_file_path, 'r') as file:
                                return file.read()

def main(args=None):
        rclpy.init(args=args)
        node = MultiRobotSpawner()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()