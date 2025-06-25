#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped

class ArucoFollower(Node):
    def __init__(self):
        super().__init__('aruco_follower')
        self.subscriber = self.create_subscription(String, '/aruco_position', self.callback, 10)
        self.publisher = self.create_publisher(TwistStamped, '/rover_controller/cmd_vel', 10)   # uses twiststamped

    def callback(self, msg):
        self.get_logger().info(f"Received direction: {msg.data}")
        twist_stamped = TwistStamped()

        # Fill the header with timestamp and frame_id
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'  # You can change this frame as needed

        # Populate the twist message
        if msg.data == "left":
            twist_stamped.twist.angular.z = 0.5 
            twist_stamped.twist.linear.x = 0.0

        elif msg.data == "right":
            twist_stamped.twist.angular.z = -0.5
            twist_stamped.twist.linear.x = 0.0

        elif msg.data == "center":
            twist_stamped.twist.angular.z = 0.0
            twist_stamped.twist.linear.x = 0.5
        else:
            # default - turns on itself until it sees a marker
            twist_stamped.twist.angular.z = 0.5
            twist_stamped.twist.linear.x = 0.0

        self.publisher.publish(twist_stamped)
        self.get_logger().info(f"Published rover cmd_vel: {twist_stamped}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
