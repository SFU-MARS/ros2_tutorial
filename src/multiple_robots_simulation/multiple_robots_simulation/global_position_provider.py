import rclpy
import numpy as np
from rclpy.node import Node




class GlobalPositionProvider(Node):
    def __init__(self):
        super().__init__('global_position_provider')
        
        self.robot_count = 3
        self.global_frame = 'map'
        self.update_rate = 10.0
        

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPositionProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()