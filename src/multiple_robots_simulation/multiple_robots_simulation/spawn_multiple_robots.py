from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class MultiRobotSpawner(Node):
        def __init__(self):
                super().__init__('multi_robot_spawner')
                self.robots = []
                self.robot_count = 3
                self.config = {
                        'robot_colors': ['Blue', 'Red', 'Green'],
                        'robot_positions': [
                                [0.0, 0.0, 0.15],
                                [1.0, 0.0, 0.15],
                                [0.0, 1.0, 0.15]
                        ]
                }

                self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
                while not self.spawn_client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info("waiting")

