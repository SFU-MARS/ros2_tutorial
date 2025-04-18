#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import signal
import sys

from .grid_map import GridMap, CellStatus
from .mcts import MCTS

class CleanerNode(Node):
    def __init__(self):
        super().__init__('cleaner_node')
        
        # Get parameters
        self.map_width = 100  # Will be updated from map message
        self.map_height = 100  # Will be updated from map message
        self.resolution = 0.05  # Matches world_map.yaml resolution
        self.mcts_iterations = 100
        self.mcts_exploration_weight = 1.0
        self.mcts_simulation_steps = 20
        self.clean_threshold = 80.0
        self.robot_radius = 0.5
        self.linear_speed = 0.02
        self.angular_speed = 0.1
        self.count = 0
        
        # Initialize grid map with default size (will be updated when map is received)
        self.grid_map = GridMap(self.map_width, self.map_height, self.resolution, self)
        
        # Initialize MCTS
        self.mcts = MCTS(
            exploration_weight=self.mcts_exploration_weight,
            simulation_steps=self.mcts_simulation_steps,
            num_iterations=self.mcts_iterations,
            robot_radius=self.robot_radius
        )
        
        # Robot state
        self.robot_pose = None  # (x, y, theta) in real world coordinates
        self.robot_grid_pos = None  # (x, y) in grid coordinates
        self.target_pose = None  # Target pose for the robot
        
        signal.signal(signal.SIGINT, self.handle_shutdown)
        
        # Create subscribers with larger queue sizes
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10  # Increased queue size
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # Increased queue size
        )
        
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.scan_callback,
        #     10  # Increased queue size
        # )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.grid_publisher = self.create_publisher(
            MarkerArray,
            '/grid_visualization',
            10
        )
        
        self.path_publisher = self.create_publisher(
            Marker,
            '/path_visualization',
            10
        )
        
        # Initialization flags
        self.map_initialized = False
        self.robot_initialized = False
        
        # Create a timer for the main control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Create a timer for visualization
        self.visualization_timer = self.create_timer(0.1, self.publish_visualization)
        
        self.get_logger().info('Cleaner node initialized')


    def handle_shutdown(self, signum, frame):
        self.get_logger().info('Shutdown signal received. Stopping robot...')
        self.send_velocity(0.0, 0.0)
        rclpy.shutdown()
        sys.exit(0)
    

    def map_callback(self, msg):
        self.get_logger().info('Received map message')
        
        # Update map dimensions from the received map
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        
        # Update grid map with new map data
        self.grid_map.update_from_occupancy_grid(msg)
        self.map_initialized = True
        self.get_logger().info(f'Map updated successfully with dimensions: {self.map_width}x{self.map_height}x{self.resolution}')
    

    def odom_callback(self, msg):
        # Extract position and orientation from odometry
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        yaw = 2.0 * math.atan2(orientation.z, orientation.w)
        
        # Update robot pose
        self.robot_pose = (position.x, position.y, yaw)
        

        # Update grid position
        self.robot_grid_pos = self.grid_map.world_to_grid(position.x, position.y)
        # x, y = self.grid_map.grid_to_world(self.robot_grid_pos[0], self.robot_grid_pos[1])
        # self.robot_pose = (x, y, yaw)
        self.get_logger().info(f'Robot pose received: {self.robot_pose}, Grid position: {self.robot_grid_pos}')
        
        # Mark the current position as cleaned
        if self.map_initialized:
            self.grid_map.mark_cleaned(position.x, position.y)
        
        self.robot_initialized = True
    
    # def scan_callback(self, msg):
    #     if not self.robot_initialized:
    #         return
        
    #     # Get robot position
    #     x, y, theta = self.robot_pose
        
    #     # Process each laser scan ray
    #     for i, range_val in enumerate(msg.ranges):
    #         # Skip invalid measurements
    #         if not math.isfinite(range_val) or range_val > msg.range_max or range_val < msg.range_min:
    #             continue
            
    #         # Calculate angle of the ray
    #         angle = msg.angle_min + i * msg.angle_increment
            
    #         # Calculate world coordinates of the endpoint
    #         obstacle_x = x + range_val * math.cos(theta + angle)
    #         obstacle_y = y + range_val * math.sin(theta + angle)
            
    #         # Update the grid cell as an obstacle
    #         self.grid_map.update_cell(obstacle_x, obstacle_y, CellStatus.OBSTACLE, self)
    
    def control_loop(self):
        self.get_logger().info('Control loop started')
        if not self.map_initialized or not self.robot_initialized:
            # Provide more detailed initialization status
            if not self.map_initialized:
                self.get_logger().info('Waiting for map initialization...')
            if not self.robot_initialized:
                self.get_logger().info('Waiting for robot pose initialization...')
            return
        
        # # Display the current robot position and grid position for debugging
        # if self.robot_pose and self.robot_grid_pos:
        #     self.get_logger().info(
        #         'Robot position: ({:.16f}, {:.16f}, {:.16f}), Grid position: ({}, {})'.format(
        #             self.robot_pose[0], self.robot_pose[1], self.robot_pose[2],
        #             int(self.robot_grid_pos[0]), int(self.robot_grid_pos[1])
        #         )
        #     )

        # Check if cleaning is complete
        coverage = self.grid_map.get_cleaning_coverage()
        if coverage >= self.clean_threshold:
            self.get_logger().info('Cleaning complete! Coverage: {:.3f}%'.format(coverage))
            # Stop the robot
            self.send_velocity(0.0, 0.0)
            return
        else:
            self.get_logger().info('Cleaning Coverage: {:.3f}%'.format(coverage))
        
        # Check if we need to compute a new target
        if self.target_pose is None:
            self.compute_next_target()
        
        # If we have a target, move towards it
        if self.target_pose:
            self.move_to_target()
    
    def compute_next_target(self):
        """Use MCTS to compute the next target cell"""
        self.get_logger().info('Computing next target')
        # Prepare the state for MCTS
        robot_grid_pos = (int(self.robot_grid_pos[0]), int(self.robot_grid_pos[1]))
        state = self.mcts.prepare_state(self.grid_map, robot_grid_pos)
        
        # Run MCTS to find the best action
        action = self.mcts.search(state)
        
        if action is None:
            self.get_logger().warn('No valid action found!')
            return
        else:
            if action == (-1, 0):
                self.get_logger().error('Action: Moving left')
            elif action == (0, 1):
                self.get_logger().error('Action: Moving up')
            elif action == (0, -1):
                self.get_logger().error('Action: Moving down')
            elif action == (1, 0):
                self.get_logger().error('Action: Moving right')
            else:
                self.get_logger().error('Invalid action!')
                return
        
        # Apply the action to get the target grid position
        dx, dy = action
        target_grid_x = int(self.robot_grid_pos[0] + dx)
        target_grid_y = int(self.robot_grid_pos[1] + dy)
        
        # Convert to world coordinates
        target_x, target_y = self.grid_map.grid_to_world(target_grid_x, target_grid_y)
        self.get_logger().error(f'robot pose: {self.robot_pose} robot grid pos: {self.robot_grid_pos}, target grid pos: {target_grid_x}, {target_grid_y}, target pose: {target_x}, {target_y}')
        
        # Set the target pose
        self.target_pose = (target_x, target_y)
        self.get_logger().info('New target: ({}, {}), Grid position: ({}, {})'.format(target_x, target_y, target_grid_x, target_grid_y))
    
    def move_to_target(self):
        self.get_logger().info('Moving...')
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
        
        # # Check for obstacles in the path
        # obstacle_in_path = self.check_path_for_obstacles(x, y, target_x, target_y)
        
        # If reached the target
        if distance < self.resolution:
            self.get_logger().info('Reached target')
            self.target_pose = None
            self.send_velocity(0.0, 0.0)
            return
        
        # # If there's an obstacle in the path, compute a new target
        # if obstacle_in_path:
        #     self.get_logger().info('Obstacle detected in path, finding new target')
        #     self.target_pose = None
        #     self.send_velocity(0.0, 0.0)
        #     return
        
        # First, rotate to face the target
        if abs(angle_diff) > 0.2:
            # Rotate in place
            angular_z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            self.send_velocity(0.0, angular_z)
            self.get_logger().info(f'Rotating to face the target: {angle_diff}')
        else:
            # Move forward
            self.send_velocity(self.linear_speed, 0.0)
            self.get_logger().info(f'Moving forward: {distance}')

    # def check_path_for_obstacles(self, start_x, start_y, target_x, target_y):
    #     # Convert world coordinates to grid coordinates
    #     start_grid_x, start_grid_y = self.grid_map.world_to_grid(start_x, start_y)
    #     target_grid_x, target_grid_y = self.grid_map.world_to_grid(target_x, target_y)
        
    #     # Use Bresenham's line algorithm to check cells along the path
    #     cells = self.bresenham_line(start_grid_x, start_grid_y, target_grid_x, target_grid_y)
        
    #     # Check each cell in the path
    #     for grid_x, grid_y in cells:
    #         if 0 <= grid_x < self.grid_map.grid_width and 0 <= grid_y < self.grid_map.grid_height:
    #             # Check if cell is an obstacle
    #             if self.grid_map.grid[grid_y, grid_x] == CellStatus.OBSTACLE.value:
    #                 self.get_logger().info('Obstacle detected')
    #                 return True
                
    #             # Also check cells around the robot (accounting for robot radius)
    #             radius_cells = int(self.robot_radius / self.resolution) + 1
    #             for i in range(-radius_cells, radius_cells + 1):
    #                 for j in range(-radius_cells, radius_cells + 1):
    #                     check_x, check_y = grid_x + i, grid_y + j
    #                     if (0 <= check_x < self.grid_map.grid_width and 
    #                         0 <= check_y < self.grid_map.grid_height and
    #                         self.grid_map.grid[check_y, check_x] == CellStatus.OBSTACLE.value):
    #                         self.get_logger().info('Obstacle detected at ({}, {})'.format(check_x, check_y))
    #                         return True
    #     return False
    
    # def bresenham_line(self, x0, y0, x1, y1):
    #     """Bresenham's line algorithm for getting cells in a line"""
    #     cells = []
    #     dx = abs(x1 - x0)
    #     dy = abs(y1 - y0)
    #     sx = 1 if x0 < x1 else -1
    #     sy = 1 if y0 < y1 else -1
    #     err = dx - dy
        
    #     while True:
    #         cells.append((x0, y0))
    #         if x0 == x1 and y0 == y1:
    #             break
                
    #         e2 = 2 * err
    #         if e2 > -dy:
    #             err -= dy
    #             x0 += sx
    #         if e2 < dx:
    #             err += dx
    #             y0 += sy
                
    #     return cells
    
    def send_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    
    def publish_visualization(self):
        if not self.map_initialized or not self.robot_initialized:
            return

        # Offset to center the map around the odom (robot becomes the origin)
        robot_x, robot_y, _ = self.robot_pose
        offset_x = -robot_x
        offset_y = -robot_y

        grid_markers = MarkerArray()
        marker_id = 0

        # Ensure we stay within grid bounds
        for x in range(self.grid_map.grid_width):
            for y in range(self.grid_map.grid_height):
                cell = self.grid_map.grid[y][x]

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "grid"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                world_x, world_y = self.grid_map.grid_to_world(x, y)
                marker.pose.position.x = world_x + offset_x
                marker.pose.position.y = world_y + offset_y
                marker.pose.position.z = 0.0
                marker.scale.x = self.resolution
                marker.scale.y = self.resolution
                marker.scale.z = 0.01

                # Your original colors
                if cell == 1:  # Obstacle
                    marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
                elif cell == 2:  # Frontier
                    marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)
                elif cell == 3:  # Visited
                    marker.color = ColorRGBA(r=0.5, g=1.0, b=0.0, a=0.5)
                else:  # Free space
                    marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.1)

                marker.lifetime.sec = 2
                grid_markers.markers.append(marker)
                marker_id += 1

        # Robot marker (color unchanged)
        robot_marker = Marker()
        robot_marker.header.frame_id = "map"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 9998
        robot_marker.type = Marker.CUBE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = self.robot_pose[0] + offset_x
        robot_marker.pose.position.y = self.robot_pose[1] + offset_y
        robot_marker.pose.position.z = 0.1
        robot_marker.scale.x = self.resolution
        robot_marker.scale.y = self.resolution
        robot_marker.scale.z = 0.01
        robot_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan (if this was also yours)
        robot_marker.lifetime.sec = 2
        grid_markers.markers.append(robot_marker)

        # Target marker (if exists)
        if self.target_pose:
            target_marker = Marker()
            target_marker.header.frame_id = "map"
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = "target"
            target_marker.id = 9999
            target_marker.type = Marker.CUBE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = self.target_pose[0] + offset_x
            target_marker.pose.position.y = self.target_pose[1] + offset_y
            target_marker.pose.position.z = 0.1
            target_marker.scale.x = self.resolution
            target_marker.scale.y = self.resolution
            target_marker.scale.z = 0.01
            target_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow (if this was yours too)
            target_marker.lifetime.sec = 2
            grid_markers.markers.append(target_marker)

        # Publish
        self.grid_publisher.publish(grid_markers)

    
    # def publish_visualization(self):
    #     if not self.map_initialized:
    #         return
        
    #     # Create a marker array for the grid
    #     grid_markers = MarkerArray()
        
    #     # Add markers for each cell type
    #     marker_id = 0
        
    #     for y in range(self.grid_map.grid_height):
    #         for x in range(self.grid_map.grid_width):
    #             # Skip unknown cells
    #             if self.grid_map.grid[y, x] == CellStatus.UNKNOWN.value:
    #                 continue
                
    #             # Create a marker for the cell
    #             marker = Marker()
    #             marker.header.frame_id = "map"  # Use the map frame for cartographer compatibility
    #             marker.header.stamp = self.get_clock().now().to_msg()
    #             marker.ns = "grid"
    #             marker.id = marker_id
    #             marker_id += 1
    #             marker.type = Marker.CUBE
    #             marker.action = Marker.ADD
                
    #             # Set position
    #             world_x, world_y = self.grid_map.grid_to_world(x, y)
    #             marker.pose.position.x = world_x #- (self.resolution / 2)
    #             marker.pose.position.y = world_y #- (self.resolution / 2)
    #             marker.pose.position.z = 0.1  # Closer to the ground to be under the map
                
    #             # Set orientation (identity quaternion)
    #             marker.pose.orientation.w = 1.0
                
    #             # Set scale (slightly smaller than cell size)
    #             scale = self.resolution
    #             marker.scale.x = scale
    #             marker.scale.y = scale
    #             marker.scale.z = 0.005 
                
    #             # Set color based on cell status
    #             color = ColorRGBA()
    #             if self.grid_map.grid[y, x] == CellStatus.OBSTACLE.value:
    #                 # Red for obstacles
    #                 color.r = 1.0
    #                 color.g = 0.0
    #                 color.b = 0.0
    #                 color.a = 0.5
    #             elif self.grid_map.grid[y, x] == CellStatus.CLEANED.value:
    #                 # Green for cleaned cells
    #                 color.r = 0.0
    #                 color.g = 1.0
    #                 color.b = 0.0
    #                 color.a = 0.5
    #             elif self.grid_map.grid[y, x] == CellStatus.FREE.value:
    #                 # Blue for free cells
    #                 color.r = 0.0
    #                 color.g = 0.0
    #                 color.b = 1.0
    #                 color.a = 0.3
                
    #             marker.color = color
    #             # Add lifetime to markers
    #             marker.lifetime.sec = 2
                
    #             # Add to array
    #             grid_markers.markers.append(marker)
        
        


    #     # TODO
    #     # Visualize the robot position as a cyan sphere
    #     if self.robot_pose is not None:
    #         robot_marker = Marker()
    #         robot_marker.header.frame_id = "map"
    #         robot_marker.header.stamp = self.get_clock().now().to_msg()
    #         robot_marker.ns = "robot_position"
    #         robot_marker.id = 9998
    #         robot_marker.type = Marker.SPHERE
    #         robot_marker.action = Marker.ADD
    #         robot_marker.pose.position.x = self.robot_pose[0]
    #         robot_marker.pose.position.y = self.robot_pose[1]
    #         robot_marker.pose.position.z = 0.05
    #         robot_marker.pose.orientation.w = 1.0
    #         robot_marker.scale.x = self.resolution
    #         robot_marker.scale.y = self.resolution
    #         robot_marker.scale.z = 0.02
    #         robot_marker.color.r = 0.0
    #         robot_marker.color.g = 1.0
    #         robot_marker.color.b = 1.0  # Cyan = green + blue
    #         robot_marker.color.a = 1.0
    #         robot_marker.lifetime.sec = 2
    #         grid_markers.markers.append(robot_marker)

    #     # Visualize the target position as a yellow sphere
    #     if self.target_pose is not None:
    #         target_marker = Marker()
    #         target_marker.header.frame_id = "map"
    #         target_marker.header.stamp = self.get_clock().now().to_msg()
    #         target_marker.ns = "target_position"
    #         target_marker.id = 9999
    #         target_marker.type = Marker.SPHERE
    #         target_marker.action = Marker.ADD
    #         target_marker.pose.position.x = self.target_pose[0]
    #         target_marker.pose.position.y = self.target_pose[1]
    #         target_marker.pose.position.z = 0.05
    #         target_marker.pose.orientation.w = 1.0
    #         target_marker.scale.x = self.resolution
    #         target_marker.scale.y = self.resolution
    #         target_marker.scale.z = 0.02
    #         target_marker.color.r = 1.0
    #         target_marker.color.g = 1.0
    #         target_marker.color.b = 0.0  # Yellow = red + green
    #         target_marker.color.a = 1.0
    #         target_marker.lifetime.sec = 2
    #         grid_markers.markers.append(target_marker)

    #     # Make sure we're not sending an empty array
    #     if grid_markers.markers:
    #         # Publish grid markers
    #         self.grid_publisher.publish(grid_markers)

        # # Visualize the robot position and orientation
        # if self.robot_pose is not None:
        #     # Create a marker for the robot position and orientation
        #     robot_marker = Marker()
        #     robot_marker.header.frame_id = "map"  # Use the map frame for cartographer compatibility
        #     robot_marker.header.stamp = self.get_clock().now().to_msg()
        #     robot_marker.ns = "robot"
        #     robot_marker.id = 0
        #     robot_marker.type = Marker.ARROW  # Arrow to show orientation
        #     robot_marker.action = Marker.ADD
            
        #     # Set robot position
        #     robot_marker.pose.position.x = self.robot_pose[0]
        #     robot_marker.pose.position.y = self.robot_pose[1]
        #     robot_marker.pose.position.z = 0.1  # Slightly above the grid markers
            
        #     # Set robot orientation using our custom euler to quaternion conversion
        #     yaw = self.robot_pose[2]
        #     quat = self.euler_to_quaternion(0.0, 0.0, yaw)
        #     robot_marker.pose.orientation.x = quat[0]
        #     robot_marker.pose.orientation.y = quat[1]
        #     robot_marker.pose.orientation.z = quat[2]
        #     robot_marker.pose.orientation.w = quat[3]
            
        #     # Set arrow size
        #     robot_marker.scale.x = 0.3  # Arrow length
        #     robot_marker.scale.y = 0.05  # Arrow width
        #     robot_marker.scale.z = 0.05  # Arrow height
            
        #     # Set color (yellow)
        #     robot_marker.color.r = 1.0
        #     robot_marker.color.g = 1.0
        #     robot_marker.color.b = 0.0
        #     robot_marker.color.a = 1.0
            
        #     # Add lifetime to marker
        #     robot_marker.lifetime.sec = 2
            
        #     # Publish robot position marker
        #     self.path_publisher.publish(robot_marker)
        
        # # Visualize the target if available
        # if self.target_pose is not None and self.robot_pose is not None:
        #     # Create a line strip marker from robot to target
        #     path_marker = Marker()
        #     path_marker.header.frame_id = "map"  # Use the map frame for cartographer compatibility
        #     path_marker.header.stamp = self.get_clock().now().to_msg()
        #     path_marker.ns = "path"
        #     path_marker.id = 0
        #     path_marker.type = Marker.LINE_STRIP
        #     path_marker.action = Marker.ADD
            
        #     # Set path points (robot and target)
        #     p1 = Point()
        #     p1.x = self.robot_pose[0]
        #     p1.y = self.robot_pose[1]
        #     p1.z = 0.05  # Slightly above the grid markers
            
        #     p2 = Point()
        #     p2.x = self.target_pose[0]
        #     p2.y = self.target_pose[1]
        #     p2.z = 0.05  # Slightly above the grid markers
            
        #     path_marker.points = [p1, p2]
            
        #     # Set line properties
        #     # path_marker.scale.x = 0.05  # Line width
        #     scale = self.resolution
        #     path_marker.scale.x = scale
        #     path_marker.scale.y = scale
        #     path_marker.scale.z = 0.005 
            
        #     # Set color (yellow)
        #     path_marker.color.r = 1.0
        #     path_marker.color.g = 1.0
        #     path_marker.color.b = 0.0
        #     path_marker.color.a = 1.0
            
            
        #     # Publish path marker
        #     self.path_publisher.publish(path_marker)
    
    # def euler_to_quaternion(self, roll, pitch, yaw):
    #     # Calculate trig values
    #     cy = math.cos(yaw * 0.5)
    #     sy = math.sin(yaw * 0.5)
    #     cp = math.cos(pitch * 0.5)
    #     sp = math.sin(pitch * 0.5)
    #     cr = math.cos(roll * 0.5)
    #     sr = math.sin(roll * 0.5)
        
    #     # Calculate quaternion components
    #     qx = sr * cp * cy - cr * sp * sy
    #     qy = cr * sp * cy + sr * cp * sy
    #     qz = cr * cp * sy - sr * sp * cy
    #     qw = cr * cp * cy + sr * sp * sy
        
    #     return [qx, qy, qz, qw]
    

def main(args=None):
    rclpy.init(args=args)
    
    cleaner_node = CleanerNode()
    
    try:
        rclpy.spin(cleaner_node)
    except KeyboardInterrupt:
        cleaner_node.handle_shutdown(signal.SIGINT, None)
    
    cleaner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 

# source ~/.bashrc && colcon build --packages-select turtlebot3_cleaner && source install/setup.bash
# source install/setup.bash && source ~/.bashrc && ros2 launch turtlebot3_cleaner cleaner.launch.py

# source ~/.bashrc && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# source ~/.bashrc && ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=world_map.yaml

# source ~/.bashrc && ros2 run turtlebot3_teleop teleop_keyboard 

# source ~/.bashrc && ros2 run rviz2 rviz2 -d ~/ros/my_config.rviz


