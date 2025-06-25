#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Note: CMake File needs to list te programs files

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',    # camera feed
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(String, '/aruco_position', 10)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)       # in simuation we are using a 6x6 marker but for compt it is a 5x5 marker do this should change
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for corner in corners:
                x = corner[0][0][0]
                width = cv_image.shape[1]
                center = width // 2
                # positions of the marker
                direction = "center"
                if x < center - (width // 4):
                    direction = "left"
                elif x > center + (width // 4):
                    direction = "right"

                self.get_logger().info(f"Marker detected. Position: {direction}")

                msg = String()
                msg.data = direction
                self.publisher_.publish(msg)

        else:
            # If no marker is detected
            self.get_logger().info("No marker detected. Publishing 'none'")
            msg = String()
            msg.data = "none"
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
