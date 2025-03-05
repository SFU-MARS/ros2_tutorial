import os
import rclpy
import yaml
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import time
import xml.etree.ElementTree as ET

class MultiRobotSpawner(Node):
        def __init__(self):
                super().__init__('multi_robot_spawner')
                self.robots = []

                self.declare_parameter('config_file', 'robot_config.yaml')
                self.declare_parameter('urdf_file', 'box_bot.urdf')

                config_file = self.get_parameter('config_file').get_parameter_value().string_value
                package_dir = get_package_share_directory('multiple_robots_simulation')
                self.urdf_file = self.get_parameter('urdf_file').get_parameter_value().string_value

                try:
                        config_path = os.path.join(package_dir, 'config', config_file)
                        with open(config_path, 'r') as f:
                                self.config = yaml.safe_load(f)
                        self.get_logger().info(f'Loaded configuration from {config_path}')
                except Exception as e:
                        self.get_logger().warn(f'Could not load config file: {e}')
                        self.config = {
                                'robot_count': "2",
                                'robot_positions': [
                                        [0.0, 0.0, 0.0],
                                        [1.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0],
                                        [1.0, 1.0, 0.0],
                                        [-1.0, 0.0, 0.0],
                                        [0.0, -1.0, 0.0],
                                        [-1.0, -1.0, 0.0],
                                        [1.0, -1.0, 0.0]
                                ]
                        }
                
                self.robot_count = min(int(self.config['robot_count']) , len(self.config['robot_positions']))

                self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
                while not self.spawn_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info("waiting")

                self.spawn_robots()

        def spawn_robots(self):
                package_dir = get_package_share_directory('multiple_robots_simulation')
                urdf_path = os.path.join(package_dir, 'models', self.urdf_file)
                
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

                                if ros_element is None:
                                        ros_element = ET.SubElement(diff_drive_plugin, 'ros')
                                
                                # Add namespace element and tf remapping
                                namespace_element = ET.SubElement(ros_element, 'namespace')
                                namespace_element.text = '/' + robot_namespace
                                remap_element = ET.SubElement(ros_element, 'remapping')
                                remap_element.text = '/tf:=/' + robot_namespace + '/tf'
                                
                                # Update odometry frames
                                odometry_frame = diff_drive_plugin.find('./odometry_frame')
                                if odometry_frame is not None:
                                        odometry_frame.text = robot_namespace + '/odom'
                                robot_base_frame = diff_drive_plugin.find('./robot_base_frame')
                                if robot_base_frame is not None:
                                        robot_base_frame.text = robot_namespace + '/chassis'
                                
                                # Update topic names
                                cmd_vel_topic = diff_drive_plugin.find('./cmd_vel_topic')
                                if cmd_vel_topic is not None:
                                        cmd_vel_topic.text = robot_namespace + '/cmd_vel'
                                
                                odometry_topic = diff_drive_plugin.find('./odometry_topic')
                                if odometry_topic is not None:
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
        node.get_logger().info("generate all the robot")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()