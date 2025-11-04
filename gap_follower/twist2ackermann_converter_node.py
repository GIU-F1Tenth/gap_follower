import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import math

class AckermannToTwist(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')
        self.declare_parameter('wheelbase', 0.33)
        self.wheelbase = self.get_parameter('wheelbase').value

        self.sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.callback,
            10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def callback(self, msg):
        t = Twist()
        t.linear.x = msg.drive.speed
        t.angular.z = msg.drive.speed * math.tan(msg.drive.steering_angle) / self.wheelbase
        self.pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
