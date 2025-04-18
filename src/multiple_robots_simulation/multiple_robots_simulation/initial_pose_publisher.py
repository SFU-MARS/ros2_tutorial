import rclpy
from rclpy.node import Node
import yaml
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.declare_parameter('config_file', 'robot_config_lab.yaml')
        
        config_file = self.get_parameter('config_file').value
        self.get_logger().info(f'Loading robot configuration from: {config_file}')
        
        try:
            config_path = os.path.join(get_package_share_directory('multiple_robots_simulation'),'config',config_file)
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load config file: {e}')
            return
            
        self.pose_publishers = {}
        for robot in self.config['robots']:
            robot_id = robot['id']
            topic = f'/tb_{robot_id}/initialpose'
            self.pose_publishers[robot_id] = self.create_publisher(
                PoseWithCovarianceStamped, topic, 10
            )
            
        time.sleep(2.0)
        
        # Publish initial poses
        self.publish_initial_poses()
        
    def publish_initial_poses(self):
        for robot in self.config['robots']:
            robot_id = robot['id']
            pose = robot['initial_pose']
            
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            # Set position & orientation (convert yaw to quaternion)
            msg.pose.pose.position.x = float(pose['x'])
            msg.pose.pose.position.y = float(pose['y'])
            msg.pose.pose.position.z = 0.0
            
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = float(pose['oz'])
            msg.pose.pose.orientation.w = float(pose['ow'])
            
            # Set covariance (medium certainty)
            covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            msg.pose.covariance = covariance
            
            # Publish
            self.pose_publishers[robot_id].publish(msg)
            self.get_logger().info(f'Published initial pose for robot {robot_id}')
        
        self.get_logger().info('All initial poses published. Shutting down.')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node) 
    rclpy.shutdown()

if __name__ == '__main__':
    main()