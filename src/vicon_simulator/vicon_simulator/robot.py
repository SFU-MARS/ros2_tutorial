import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class Robot:
    v = 1.0
    def __init__(self) -> None:
        self.z = np.array([0.0, 0.0, 0.0])

    def update_state(self, dt):
        dx = self.v * np.cos(self.z[2])
        dy = self.v * np.sin(self.z[2])
        dtheta = 1.0

        z_tp1 = self.z + dt * np.array([dx, dy, dtheta])
        self.z = z_tp1

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class RobotPoseSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_pose_simulator')
        self.create_timer(0.1, self.update_pose)
        self.pub = self.create_publisher(PoseStamped, '/robot_pose', 1)
        self.robot = Robot()
    
    def update_pose(self):
        self.robot.update_state(0.1)

        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        point = Point(x=self.robot.z[0], y=self.robot.z[1], z=0.5)

        q = quaternion_from_euler(0.0, 0.0, self.robot.z[2])
        orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        msg.pose.position = point
        msg.pose.orientation = orientation

        self.get_logger().info(f'Robot position: x={self.robot.z[0]:0.2f} y={self.robot.z[1]:0.2f} theta={self.robot.z[2]:0.2f}')
        self.pub.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseSimulatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
