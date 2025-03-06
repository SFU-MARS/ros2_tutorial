from rclpy.node import Node
import rclpy
import os
import numpy as np
import yaml
import threading
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from . import robot


class BVCController(Node):
        def __init__(self):
                super().__init__('bvc_controller')

                self.safety_radius = 0.3
                self.max_linear_speed = 0.2
                robot_config = "robot_config.yaml"
                world_size = 15
                self.max_angular_speed = 0.5
                self.update_rate = 10

                self.world_corners = np.array([
                [-world_size, -world_size, world_size, world_size], 
                [-world_size, world_size, world_size, -world_size]
                ])


                try:
                        package_dir = get_package_share_directory('multiple_robots_simulation')
                        config_path = os.path.join(package_dir, 'config', robot_config)
                        with open(config_path, 'r') as f:
                                config = yaml.safe_load(f)
                        self.get_logger().info(f'Loaded goals from {config_path}')

                        self.robot_count = min(int(config['robot_count']) ,len(config['robot_positions']))
                        self.goals = config['robot_goals']
                                
                except Exception as e:
                        self.get_logger().warning(f'Could not load goals config: {e}')
                        # Use default goals if config file is not available
                        self.robot_count = 2
                        self.goals = np.array([
                                [3.0, 3.0],    
                                [-3.0, 3.0]  
                        ])

                # Initialize robot positions, velocities, and goals
                self.positions = np.zeros((self.robot_count, 2))
                self.velocities = np.zeros((self.robot_count, 2))

                self.bvc_robots = []
                for i in range(self.robot_count):
                        r = robot.Robot(i)
                        self.bvc_robots.append(r)

                self.odom_lock = threading.Lock()
                self.odom_received = [False] * self.robot_count

                self.robots_detected = False
                self.initialized = False

                self.odom_subscribers = []
                #check the namespace of different robots
                for i in range(self.robot_count):
                        sub = self.create_subscription(
                                Odometry,
                                f'/tb_{i}/odom',
                                lambda msg, idx=i: self.odom_callback(msg, idx),
                                10
                        )
                        self.odom_subscribers.append(sub)

                self.velocity_pubs = []
                for i in range(self.robot_count):
                        pub = self.create_publisher(
                                Twist, 
                                f'/tb_{i}/cmd_vel', 
                                10
                        )
                        self.velocity_pubs.append(pub)

                self.goals_reached = [False] * self.robot_count
                self.control_timer = None

                self.detection_timer = self.create_timer(1.0, self.wait_for_robots)


        def odom_callback(self, msg, robot_idx):
                with self.odom_lock:
                        self.positions[robot_idx, 0] = msg.pose.pose.position.x
                        self.positions[robot_idx, 1] = msg.pose.pose.position.y
                        self.odom_received[robot_idx] = True

        def wait_for_robots(self):
                if self.robots_detected:
                        return
            
                with self.odom_lock:
                        all_robots_found = all(self.odom_received)
                        detected_count = sum(self.odom_received)
        
                # Report finding robot
                if not all_robots_found:
                        self.get_logger().info(f'Waiting for robots... ({detected_count}/{self.robot_count} detected)')
                else:
                        self.get_logger().info(f'All {self.robot_count} robots detected!')
                        self.robots_detected = True
                        self.detection_timer.cancel()
                        
                        self.control_timer = self.create_timer(1.0/self.update_rate, self.control_loop)

        def initialize_bvc_robots(self):
                for i in range(self.robot_count):
                # Set BVC with current position
                        pos = self.positions[i]
                        self.bvc_robots[i].set_bvc(pos, self.safety_radius, self.world_corners)
                        self.bvc_robots[i].set_goal(self.goals[i])
                
                self.initialized = True
                self.get_logger().info('BVC robots initialized with current positions')


        def control_loop(self):
                """Main control loop"""
                if not self.robots_detected:
                        return
                
                if not self.initialized:
                        self.initialize_bvc_robots()
                        return 
                



def main(args=None):
    rclpy.init(args=args)
    controller = BVCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
