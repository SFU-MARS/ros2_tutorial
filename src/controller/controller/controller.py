import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Vector3
import numpy as np


class ControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_controller")
        self.sub = self.create_subscription(
            PoseStamped, "/robot_pose", self.send_ctrl, 10
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.declare_parameter("algorithm", "DQN")
        algo: str = self.get_parameter("algorithm").get_parameter_value().string_value
        self.get_logger().info(f"Using algorithm {algo}")

    def send_ctrl(self, msg: PoseStamped):
        robot_x = msg.pose.position.x
        robot_y = msg.pose.position.y
        _, _, robot_theta = euler_from_quaternion(msg.pose.orientation)
        self.get_logger().info(
            f"Robot position: x={robot_x:0.2f} y={robot_y:0.2f} theta={robot_theta:0.2f}"
        )

        # Do research code

        twist_msg = Twist()
        twist_msg.linear = Vector3(x=1.0, y=0.0, z=0.0)
        twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.pub.publish(twist_msg)


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
