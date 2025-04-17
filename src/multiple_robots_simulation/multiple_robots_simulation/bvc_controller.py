from rclpy.node import Node
import rclpy
import os
import numpy as np
import yaml
import threading
import math
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray

from . import robot
import tf_transformations



class BVCController(Node):
        
        def __init__(self):
                super().__init__('bvc_controller')

                self.declare_parameter('safety_radius', 0.3)
                self.declare_parameter('update_rate', 2)
                self.declare_parameter('max_linear_speed', 0.2)
                self.declare_parameter('goal_tolerance', 0.2)
                self.declare_parameter('config_file', "robot_config_lab.yaml")
                self.declare_parameter('world_size', 15.0)
                self.declare_parameter('max_angular_speed', 0.5)
                self.declare_parameter('angle_tolerance', 0.1)

                self.safety_radius = self.get_parameter('safety_radius').value
                self.max_linear_speed = self.get_parameter('max_linear_speed').value
                robot_config = self.get_parameter('config_file').value
                world_size = self.get_parameter('world_size').value
                self.max_angular_speed = self.get_parameter('max_angular_speed').value
                self.update_rate = self.get_parameter('update_rate').value
                

                self.goal_tolerance = self.get_parameter('goal_tolerance').value
                self.angle_tolerance = self.get_parameter('angle_tolerance').value


                try:
                        package_dir = get_package_share_directory('multiple_robots_simulation')
                        config_path = os.path.join(package_dir, 'config', robot_config)
                        with open(config_path, 'r') as f:
                                config = yaml.safe_load(f)
                        self.get_logger().info(f'Loaded goals from {config_path}')

                        self.robot_count = min(int(config['robot_count']) ,len(config['robot_positions']))

                        world_edges = []
                        counter = 1
                        for edge in config["world_edge"]:
                                pose = edge[f'pose_{counter}']
                                world_edges.append([pose['x'], pose['y']])
                                counter += 1
                        self.world_corners = np.array(world_edges)

                        self.goals = []
                        for goal in config['robots']:
                                goal_pos = goal['goal']
                                self.goals.append([goal_pos['x'],goal_pos['y']])

                        # self.goals = config['robot_goals']
                                
                except Exception as e:
                        self.get_logger().warning(f'Could not load goals config: {e}')
                        # Use default goals if config file is not available
                        self.robot_count = 2
                        self.goals = np.array([
                                [3.0, 3.0],    
                                [-3.0, 3.0]  
                        ])

                        self.world_corners = np.array([
                        [-world_size, -world_size, world_size, world_size], 
                        [-world_size, world_size, world_size, -world_size]])

                # Subscribe to global positions
                self.global_positions_sub = self.create_subscription(
                        PoseArray,
                        '/global_robot_positions',
                        self.global_positions_callback,
                        10
                )

                # Initialize robot positions, velocities, and goals
                self.positions = np.zeros((self.robot_count, 2))
                self.velocities = np.zeros((self.robot_count, 2))

                self.orientations = np.zeros(self.robot_count)
                self.angular_velocities = np.zeros(self.robot_count)

                self.position_lock = threading.Lock()
                self.positions_received = False

                self.bvc_robots = []
                for i in range(self.robot_count):
                        r = robot.Robot(i)
                        self.bvc_robots.append(r)


                self.velocity_pubs = []
                for i in range(self.robot_count):
                        pub = self.create_publisher(
                                Twist, 
                                f'/tb_{i}/cmd_vel', 
                                10
                        )
                        self.velocity_pubs.append(pub)

                
                self.initialized = False
                self.positions_received = False

                self.goals_reached = [False] * self.robot_count
                self.control_timer = None

                self.detection_timer = self.create_timer(1.0, self.wait_for_robots)


        def global_positions_callback(self, msg):
                if len(msg.poses) != self.robot_count:
                        self.get_logger().warning(f'Received {len(msg.poses)} positions, expected {self.robot_count}')
                        return
                
                with self.position_lock:
                        for i in range(self.robot_count):
                                pose = msg.poses[i]
                                
                                # Store position
                                self.positions[i, 0] = pose.position.x
                                self.positions[i, 1] = pose.position.y
                                
                                q = pose.orientation
                                _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                                self.orientations[i] = yaw
                
                        self.positions_received = True

        def wait_for_robots(self):
                if self.positions_received:
                        self.get_logger().info('Global positions received, starting BVC controller')
                        self.detection_timer.cancel()
                        self.control_timer = self.create_timer(1.0/self.update_rate, self.control_loop)
                else:
                        self.get_logger().info('Waiting for global robot positions...')

        def initialize_bvc_robots(self):
                for i in range(self.robot_count):
                        # Set BVC with current position
                        pos = self.positions[i]
                        self.bvc_robots[i].set_bvc(pos, self.safety_radius, self.world_corners)
                        self.bvc_robots[i].set_goal(self.goals[i])
        
                self.initialized = True
                self.get_logger().info('BVC robots initialized with global positions')

        def update_bvc_cells(self):
                for i in range(self.robot_count):
                        if self.goals_reached[i]:
                                continue
                                
                        other_robots_indices = [j for j in range(self.robot_count) if j != i]
                        other_robots_positions = self.positions[other_robots_indices]
                        
                        # Update BVC cell
                        own_pos = self.positions[i]
                        self.bvc_robots[i].cell.update_bvc(
                                own_pos.reshape(2, 1),
                                other_robots_positions.T, 
                                np.array(other_robots_indices)
                        )
                        
                        # Update neighbor distances (for deadlock detection)
                        self.bvc_robots[i].mem_nbr_dist(
                                other_robots_positions.T,
                                np.array(other_robots_indices)
                        )


        def compute_velocities(self):
                for i in range(self.robot_count):
                        if self.goals_reached[i]:
                                # Robot that are already at goal
                                self.velocities[i] = np.zeros(2)
                                self.angular_velocities[i] = 0.0
                                continue
                        
                
                        closest = self.bvc_robots[i].bvc_find_closest_to_goal()
                
                        if closest is None:
                                self.get_logger().warning(f'No valid path found for robot_{i}')
                                self.velocities[i] = np.zeros(2)
                                self.angular_velocities[i] = 0.0
                                continue
                
                
                        closest = closest.reshape(2)
                        current = self.positions[i]
                        direction = closest - current
                
               
                        distance_to_goal = np.linalg.norm(self.goals[i] - current)
                        if distance_to_goal < self.goal_tolerance:
                                self.get_logger().info(f'Robot_{i} reached its goal!')
                                self.goals_reached[i] = True
                                self.velocities[i] = np.zeros(2)
                                self.angular_velocities[i] = 0.0
                                continue
                
                        # Calculate desired heading angle
                        desired_angle = math.atan2(direction[1], direction[0])
                        current_angle = self.orientations[i]
                        
                        # Calculate the angle difference (taking into account the wrap around)
                        angle_diff = self.normalize_angle(desired_angle - current_angle)
                
                        if abs(angle_diff) > self.angle_tolerance:
                                self.velocities[i] = np.zeros(2)
                        
                                angular_vel = self.max_angular_speed * (angle_diff / math.pi)
                                angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
                                self.angular_velocities[i] = angular_vel
                        else:
                                distance = np.linalg.norm(direction)
                                if distance > 0.001: 
                                        linear_vel = min(self.max_linear_speed, distance)
                                        self.velocities[i] = np.array([linear_vel, 0.0])  
                                        self.angular_velocities[i] = angle_diff * self.max_angular_speed
                                else:
                                        self.velocities[i] = np.zeros(2)
                                        self.angular_velocities[i] = 0.0
                                
    
        def normalize_angle(self, angle):
                while angle > math.pi:
                        angle -= 2.0 * math.pi
                while angle < -math.pi:
                        angle += 2.0 * math.pi
                return angle
        
        def publish_velocities(self):
                for i in range(self.robot_count):
                        msg = Twist()
                        
                        msg.linear.x = float(self.velocities[i][0])
                        msg.linear.y = 0.0
                        msg.linear.z = 0.0
                        
                        # Set the angular velocity for rotation
                        msg.angular.x = 0.0
                        msg.angular.y = 0.0
                        msg.angular.z = float(self.angular_velocities[i])
                        
                        self.velocity_pubs[i].publish(msg)
    

        def control_loop(self):
                """Main control loop"""
                if not self.robots_detected:
                        return
                
                if not self.initialized:
                        self.initialize_bvc_robots()
                        return 
                
                with self.position_lock:
                        self.update_bvc_cells()
                        self.compute_velocities()
                        self.publish_velocities()
            
                if all(self.goals_reached):
                        self.get_logger().info('All robots have reached their goals!')
                        if self.control_timer is not None:
                                self.control_timer.cancel()
                        self.create_timer(2.0, rclpy.shutdown)
                


def main(args=None):
    rclpy.init(args=args)
    controller = BVCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
