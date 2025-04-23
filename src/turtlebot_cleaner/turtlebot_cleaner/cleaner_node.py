#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import signal
import tf2_ros
from tf2_ros import TransformException
import rclpy.time
import rclpy.duration
from .grid_map import GridMap, CellStatus
from .mcts import MCTS



class CleanerNode(Node):
    def __init__(self):
        super().__init__('cleaner_node')
        # parameters
        self.map_width = 100
        self.map_height = 100
        self.resolution = 0.05
        self.mcts_iterations = 100
        self.mcts_exploration_weight = 1.0
        self.mcts_simulation_steps = 20
        self.clean_threshold = 80.0
        self.robot_radius = 0.2
        self.linear_speed = 0.02
        self.angular_speed = 0.05
        self.init_map = True

        self.origin = None # map origin (x, y, z)
        self.origin_x = 0.0 # Map origin x
        self.origin_y = 0.0 # Map origin y

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Grid Map
        self.grid_map = GridMap(self.map_width, self.map_height, self.resolution, self, self.origin_x, self.origin_y)

        # Initialize MCTS
        self.mcts = MCTS(
            node=self,
            exploration_weight=self.mcts_exploration_weight,
            simulation_steps=self.mcts_simulation_steps,
            num_iterations=self.mcts_iterations,
            robot_radius=self.robot_radius
        )

        # Robot state
        self.robot_pose = None
        self.robot_grid_pos = None
        self.target_pose = None

        signal.signal(signal.SIGINT, self.handle_shutdown)

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.grid_publisher = self.create_publisher(MarkerArray, '/grid_visualization', 10)

        # Initialization flags
        self.map_initialized = False
        self.robot_initialized = False

        # Create a timer for the main control loop
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # Create a timer for visualization
        self.visualization_timer = self.create_timer(0.5, self.publish_visualization)

        self.get_logger().info('Cleaner node initialized!')


    def handle_shutdown(self):
        self.get_logger().info('Shutdown signal received. Stopping robot...')
        self.send_velocity(0.0, 0.0)
        raise KeyboardInterrupt


    def map_callback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        origin = msg.info.origin
        self.origin = (origin.position.x, origin.position.y, origin.position.z)
        self.origin_x = origin.position.x
        self.origin_y = origin.position.y

        if self.init_map:
            self.grid_map = GridMap(self.map_width, self.map_height, self.resolution, self, self.origin_x, self.origin_y)
            self.init_map = False
            self.map_initialized = True
        
        self.grid_map.update_grid_from_map(msg)
        self.map_initialized = True
        self.get_logger().info(f'Map Updated!')
        

    def update_robot_pose_from_tf(self):
        try:
            # Get the latest available transform
            transform_stamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01)
            )

            # Extract position and orientation
            trans = transform_stamped.transform.translation
            rot = transform_stamped.transform.rotation
            x, y = trans.x, trans.y

            # Convert quaternion to yaw
            yaw = math.atan2(2.0 * (rot.w * rot.z + rot.x * rot.y), 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z))


            # Update robot pose (world coordinates)
            self.robot_pose = (x, y, yaw)

            # Update robot grid position
            if self.map_initialized:
                self.robot_grid_pos = self.grid_map.world_to_grid(x, y)
                self.grid_map.mark_cleaned(x, y)
                self.robot_initialized = True
                self.get_logger().info(f'Robot pose Updated!')
            else:
                 self.get_logger().warn("Cannot update robot grid position, map not initialized yet.")

        except TransformException as e:
            self.get_logger().warn(f"Could not get transform from 'map' to 'base_link': {e}")
            return False
        return True


    def control_loop(self):
        if not self.update_robot_pose_from_tf():
             self.get_logger().warn('Robot pose update failed, skipping control cycle.')
             return
        
        if not self.map_initialized or not self.robot_initialized:
            if not self.map_initialized:
                self.get_logger().info('Waiting for map initialization...')
            if not self.robot_initialized:
                self.get_logger().info('Waiting for robot pose initialization...')
            return

        coverage = self.grid_map.get_cleaning_coverage()
        if coverage >= self.clean_threshold:
            self.get_logger().info(f'Cleaning complete! Coverage: {coverage:.2f}% >= {self.clean_threshold:.2f}%')
            self.send_velocity(0.0, 0.0)
            self.control_timer.cancel()
            return

        if self.target_pose is None:
            self.compute_next_target()
        else:
            self.move_to_target()


    def compute_next_target(self):
        self.get_logger().info('Computing next target using MCTS...')

        # Prepare the state for MCTS
        robot_grid_pos_int = (int(self.robot_grid_pos[0]), int(self.robot_grid_pos[1]))
        state = self.mcts.prepare_state(self.grid_map, robot_grid_pos_int)

        # Run MCTS to find the best action
        action = self.mcts.search(state)
        
        if action is None:
            self.get_logger().warn('No valid action found!')
            self.target_pose = None
            self.send_velocity(0.0, 0.0)
            return
        else:
            if action == (-1, 0):
                direction = 'left'
                Cdx = -1.1 
                Cdy = 0.0
            elif action == (0, 1):
                direction = 'up'
                Cdx = 0.0
                Cdy = 1.1
            elif action == (0, -1):
                direction = 'down'
                Cdx = 0.0
                Cdy = -1.1
            elif action == (1, 0):
                direction = 'right'
                Cdx = 1.1
                Cdy = 0.0
            else:
                self.get_logger().info('Invalid action!')
                return

        # Apply the action to get the target grid position
        dx, dy = action
        target_grid_x = robot_grid_pos_int[0] + dx
        target_grid_y = robot_grid_pos_int[1] + dy
        target_grid_x_center = robot_grid_pos_int[0] + Cdx
        target_grid_y_center = robot_grid_pos_int[1] + Cdy

        target_x, target_y = self.grid_map.grid_to_world(target_grid_x_center, target_grid_y_center)

        # Set the target pose
        self.target_pose = (target_x, target_y)
        self.get_logger().info(f'New target computed: World=({self.target_pose[0]:3f}, {self.target_pose[1]:3f}), Grid=({robot_grid_pos_int[0]}, {robot_grid_pos_int[1]}) -> ({target_grid_x}, {target_grid_y}), Action={direction}')
    
    def move_to_target(self):
        if self.target_pose is None or self.robot_pose is None:
            return
        
        # Extract current pose
        x, y, theta = self.robot_pose
        
        # Calculate direction to target
        target_x, target_y = self.target_pose
        dx = target_x - x
        dy = target_y - y
        
        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = target_angle - theta
        
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Calculate distance to target
        distance = math.sqrt(dx * dx + dy * dy)
        
        # If reached the target
        if distance < 0.01:
            self.get_logger().info('Reached target!')
            self.grid_map.mark_cleaned(target_x, target_y)
            self.target_pose = None
            self.send_velocity(0.0, 0.0)
            return
        
        if abs(angle_diff) > 0.05:
            # Rotate in place
            angular_z = self.angular_speed if angle_diff > 0 else -self.angular_speed

            if abs(angle_diff) > math.pi / 2:
                self.send_velocity(0.0, 2 * angular_z)
            else:
                self.send_velocity(0.0, angular_z)

            self.get_logger().info(f'Rotating to face target: {angle_diff:4f} radians left')
        else:
            # Move forward
            self.send_velocity(self.linear_speed, 0.0)
            self.get_logger().info(f'Moving forward: {distance:4f} meters')


    def send_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)


    def publish_visualization(self):
        if not self.map_initialized or not self.robot_initialized or not self.grid_map:
             return

        if self.grid_map.grid is None or self.grid_map.grid_width <= 0 or self.grid_map.grid_height <= 0:
            return

        grid_markers = MarkerArray()
        marker_id = 0

        # Visualize Grid Cells
        for x in range(self.grid_map.grid_width):
            for y in range(self.grid_map.grid_height):
                if y < 0 or y >= self.grid_map.grid_height or x < 0 or x >= self.grid_map.grid_width:
                    self.get_logger().warn(f"Attempted to access invalid grid index ({x}, {y})")
                    continue

                cell_value = self.grid_map.grid[y][x]
                world_x, world_y = self.grid_map.grid_to_world(x, y)

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grid_cells"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # Assign world coordinates
                marker.pose.position.x = float(world_x)
                marker.pose.position.y = float(world_y)
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0

                # Scale based on map resolution
                marker.scale.x = float(self.resolution)
                marker.scale.y = float(self.resolution)
                marker.scale.z = 0.01

                status = CellStatus(cell_value)
                if status == CellStatus.OBSTACLE:
                    color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5) # Red: Obstacle
                elif status == CellStatus.UNKNOWN:
                    color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.1) # White: Unknown
                elif status == CellStatus.CLEANED:
                    color = ColorRGBA(r=0.5, g=1.0, b=0.0, a=0.5) # Green: Cleaned
                elif status == CellStatus.FREE:
                    color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5) # Blue: Free

                marker.color = color
                marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
                grid_markers.markers.append(marker)
                marker_id += 1

        # Visualize Robot
        if self.robot_pose:
            robot_marker = Marker()
            robot_marker.header.frame_id = "map"
            robot_marker.header.stamp = self.get_clock().now().to_msg()
            robot_marker.ns = "robot_pose"
            robot_marker.id = marker_id
            robot_marker.type = Marker.ARROW
            robot_marker.action = Marker.ADD

            # Set pose
            robot_marker.pose.position.x = float(self.robot_pose[0])
            robot_marker.pose.position.y = float(self.robot_pose[1])
            robot_marker.pose.position.z = 0.05

            # Convert yaw (self.robot_pose[2]) to quaternion
            yaw = self.robot_pose[2]
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            robot_marker.pose.orientation.x = 0.0
            robot_marker.pose.orientation.y = 0.0
            robot_marker.pose.orientation.z = sy
            robot_marker.pose.orientation.w = cy

            # Scale the arrow
            robot_marker.scale.x = self.robot_radius * 0.25 # Length of arrow
            robot_marker.scale.y = self.robot_radius * 0.1 # Width of arrow
            robot_marker.scale.z = self.robot_radius * 0.1 # Height of arrow

            robot_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0) # Cyan
            robot_marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
            grid_markers.markers.append(robot_marker)
            marker_id += 1

        # Visualize Target
        if self.target_pose:
            target_marker = Marker()
            target_marker.header.frame_id = "map"
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = "target_pose"
            target_marker.id = marker_id
            target_marker.type = Marker.CUBE
            target_marker.action = Marker.ADD

            # Set pose
            target_marker.pose.position.x = float(self.target_pose[0])
            target_marker.pose.position.y = float(self.target_pose[1])
            target_marker.pose.position.z = 0.0
            target_marker.pose.orientation.w = 1.0

            # Scale
            target_marker.scale.x = float(self.resolution)
            target_marker.scale.y = float(self.resolution)
            target_marker.scale.z = 0.01

            target_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0) # Yellow
            target_marker.lifetime = rclpy.duration.Duration(seconds=2.0).to_msg()
            grid_markers.markers.append(target_marker)
            marker_id += 1

        # Publish the complete marker array
        if grid_markers.markers:
            self.grid_publisher.publish(grid_markers)


def main(args=None):
    rclpy.init(args=args)
    cleaner_node = CleanerNode()
    try:
        rclpy.spin(cleaner_node)
    except KeyboardInterrupt:
        cleaner_node.get_logger().info('Keyboard interrupt detected, shutting down.')
    except Exception as e:
         cleaner_node.get_logger().fatal(f"Unhandled exception in main spin loop: {e}")
         import traceback
         traceback.print_exc()
    finally:
        cleaner_node.get_logger().info('Cleaning up and destroying node...')
        cleaner_node.send_velocity(0.0, 0.0)
        cleaner_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

# Build and Run Commands

# source ~/.bashrc && colcon build --packages-select turtlebot_cleaner && source install/setup.bash
# source install/setup.bash && source ~/.bashrc && ros2 launch turtlebot_cleaner cleaner.launch.py

# source ~/.bashrc && source install/setup.bash && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# source ~/.bashrc && source install/setup.bash && ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

# source ~/.bashrc && source install/setup.bash && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=world_map.yaml

# source ~/.bashrc && source install/setup.bash && ros2 run turtlebot3_teleop teleop_keyboard

# source ~/.bashrc && source install/setup.bash && ros2 run rviz2 rviz2 -d src/turtlebot3_cleaner/Myconfig.rviz
