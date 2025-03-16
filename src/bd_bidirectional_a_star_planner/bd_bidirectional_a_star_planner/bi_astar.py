import rclpy
from rclpy.node import Node

class BiAStar(Node):
    def __init__(self):
        super().__init__("bi_astar")
        self.get_logger().info("BiAStar node initialized")
      
        
def main():
    rclpy.init()
    bi_astar = BiAStar()
    rclpy.spin(bi_astar)
    bi_astar.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()





