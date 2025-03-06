from rclpy.node import Node
import rclpy
import os
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

class BVCController(Node):
    def __init__(self):
        super().__init__('bvc_controller')

        self.safety_radius = 0.3
        self.max_linear_speed = 0.2
        robot_config = "robot_config.yaml"
        world_size = 15
        self.max_angular_speed = 0.5

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


def main(args=None):
    rclpy.init(args=args)
    controller = BVCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
