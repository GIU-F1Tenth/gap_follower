import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import math


def convert_trans_rot_vel_to_steering_angle(vel, omega, wheelbase):
    if omega == 0 or vel == 0:
        return 0.0
    
    vel = abs(vel)
    rad = vel / omega
    return math.atan(wheelbase / rad)


class TwistToAckermann(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        
        self.declare_parameter('wheelbase', 0.33)
        self._wheelbase = self.get_parameter('wheelbase').value
        
        self._twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_cb, 10)
        self._ack_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_cmd', 10)

    def twist_cb(self, msg):
        out = AckermannDriveStamped()
        out.drive.speed = msg.linear.x
        out.drive.steering_angle = convert_trans_rot_vel_to_steering_angle(
            msg.linear.x, msg.angular.z, self._wheelbase)
        self._ack_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
