import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseArray, Pose



class GlobalPositionProvider(Node):
        def __init__(self):
                super().__init__('global_position_provider')
        
                self.robot_count = 3
                self.global_frame = 'map'
                self.update_rate = 10.0

                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)

                self.global_positions_pub = self.create_publisher(PoseArray, '/global_robot_positions', 10)
                self.create_timer(1.0/self.update_rate, self.publish_global_positions)

        def publish_global_positions(self):
                pose_array = PoseArray()
                pose_array.header.stamp = self.get_clock().now().to_msg()
                pose_array.header.frame_id = self.global_frame
                
                for i in range(self.robot_count):
                        robot_frame = f'tb_{i}/base_footprint'
                        try:

                                transform = self.tf_buffer.lookup_transform(self.global_frame,robot_frame,rclpy.time.Time())
                                
                                pose = Pose()
                                pose.position.x = transform.transform.translation.x
                                pose.position.y = transform.transform.translation.y
                                pose.position.z = transform.transform.translation.z
                                pose.orientation = transform.transform.rotation
                                
                                pose_array.poses.append(pose)
                        except Exception as e:
                                self.get_logger().warning(f'Could not get transform for robot {i}: {e}')

                        if len(pose_array.poses) == self.robot_count:
                                self.global_positions_pub.publish(pose_array)


        

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPositionProvider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()