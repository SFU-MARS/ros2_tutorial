import rclpy
from rclpy.node import Node
import yaml
import os
import sys
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        self.declare_parameter('config_file', 'robot_config_lab_04_21.yaml')
        
        config_file = self.get_parameter('config_file').value
        self.get_logger().info(f'Loading robot configuration from: {config_file}')
        
        try:
            config_path = os.path.join(get_package_share_directory('multiple_robots_simulation'), 'config', config_file)
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            
            # Get robot count from config
            self.robot_count = int(self.config['robot_count'])
            self.get_logger().info(f'Detected {self.robot_count} robots in configuration')
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
        
        # Create lifecycle state clients for each AMCL node
        self.state_clients = {}
        for i in range(self.robot_count):
            service_name = f'/tb_{i}/amcl_{i}/get_state'
            self.state_clients[i] = self.create_client(GetState, service_name)
            
    def is_node_active(self, node_id):
        """Check if an AMCL node is in the active state"""
        if node_id not in self.state_clients:
            self.get_logger().error(f'No state client for node {node_id}')
            return False
            
        client = self.state_clients[node_id]

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'Service not available for node {node_id}')
            return False
            
        request = GetState.Request()
        future = client.call_async(request)
        
        # Wait for response with timeout
        start_time = time.time()
        while not future.done() and time.time() - start_time < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not future.done():
            self.get_logger().warning(f'Timeout waiting for state of node {node_id}')
            return False
            
        try:
            response = future.result()
            is_active = response.current_state.id == State.PRIMARY_STATE_ACTIVE
            self.get_logger().info(f'Node {node_id} active state: {is_active}')
            return is_active
        except Exception as e:
            self.get_logger().error(f'Error getting state for node {node_id}: {e}')
            return False
    
    def wait_for_amcl_nodes(self):
        """Wait until all AMCL nodes are active or max retries is reached"""
        self.get_logger().info('Waiting for AMCL nodes to become active...')
        nodes_active = False
        retry_count = 0
        max_retries = 15  # More retries with longer total wait time
        
        while not nodes_active and retry_count < max_retries:
            try:
                active_count = 0
                for robot_id in range(self.robot_count):
                    if self.is_node_active(robot_id):
                        active_count += 1
                
                if active_count == self.robot_count:
                    nodes_active = True
                    self.get_logger().info("All AMCL nodes are active!")
                else:
                    self.get_logger().info(f"Waiting for AMCL nodes to activate ({active_count}/{self.robot_count} ready)...")
                    time.sleep(2.0)  
                    retry_count += 1
            except Exception as e:
                self.get_logger().warning(f"Error checking AMCL node states: {e}")
                time.sleep(1.0)
                retry_count += 1
        
        if not nodes_active:
            self.get_logger().warning("Timed out waiting for all AMCL nodes to activate. Proceeding anyway...")
        
        return nodes_active
        
    def publish_initial_poses(self):
        """Publish initial poses with multiple retries"""
        retry_count = 0
        max_retries = 5
        
        while retry_count < max_retries:
            self.get_logger().info(f'Publishing initial poses (attempt {retry_count+1}/{max_retries})...')
            
            for robot in self.config['robots']:
                robot_id = robot['id']
                pose = robot['initial_pose']
                
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                # Set position & orientation
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
            
            # Sleep between retries to allow AMCL to process
            time.sleep(2.0)
            retry_count += 1
            
            # Process any messages (allowing callbacks to be processed)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('All initial poses published successfully.')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create the node
        node = InitialPosePublisher()
        node.wait_for_amcl_nodes()
        node.publish_initial_poses()
        time.sleep(1.0)
        
        # Clean shutdown
        node.get_logger().info("Initial pose publisher completed its task. Shutting down.")
        node.destroy_node()
        
    except Exception as e:
        print(f"Error in initial_pose_publisher: {e}", file=sys.stderr)
    finally:
        # Ensure clean shutdown
        rclpy.shutdown()


if __name__ == '__main__':
    main()