from rclpy.node import Node
import rclpy

class BVCController(Node):
    def __init__(self):
        super().__init__('bvc_controller')


def main(args=None):
    rclpy.init(args=args)
    controller = BVCController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
