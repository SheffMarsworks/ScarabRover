import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # ✅ Add a publisher
        self.publisher_ = self.create_publisher(String, '/aruco_position', 10)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            for corner in corners:
                x = corner[0][0][0]
                width = cv_image.shape[1]
                center = width // 2

                direction = "center"
                if x < center - 50:
                    direction = "left"
                elif x > center + 50:
                    direction = "right"

                self.get_logger().info(f"Marker detected. Position: {direction}")

                # ✅ Publish direction
                msg = String()
                msg.data = direction
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
