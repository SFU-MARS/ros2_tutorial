from rclpy.node import Node
import rclpy
import numpy as np

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


def main(args=None):
    rclpy.init(args=args)
    controller = BVCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
