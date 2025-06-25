import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')
        self.subscriber = self.create_subscription(String, '/aruco_position', self.callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def callback(self, msg):
        twist = Twist()
        if msg.data == "left":
            twist.angular.z = 0.3
        elif msg.data == "right":
            twist.angular.z = -0.3
        else:
            twist.linear.x = 0.2
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
