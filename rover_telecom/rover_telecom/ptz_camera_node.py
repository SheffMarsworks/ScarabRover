import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from onvif import ONVIFCamera
from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
import os

# In your main function or class initialization
CONFIG_FILE = os.path.join(get_package_share_directory('rover_telecom'), 'config', 'ovif_config.json')


class CameraControl:
    def __init__(self, ip: str, port: int, username: str, password: str):
        # Camera setup
        self._cam = ONVIFCamera(ip, port, username, password)
        self._media_service = self._cam.create_media_service()
        self._ptz_service = self._cam.create_ptz_service()
        self._profile_token = self._media_service.GetProfiles()[0].token

    def move_camera(self, direction: str):
        pan, tilt = 0, 0
        if direction == "up":
            tilt = 1
        elif direction == "down":
            tilt = -1
        elif direction == "left":
            pan = -1
        elif direction == "right":
            pan = 1

        if pan == 0 and tilt == 0:
            self.stop_camera()
        else:
            self._move_camera(pan, tilt)

    def stop_camera(self):
        request = self._ptz_service.create_type("Stop")
        request.ProfileToken = self._profile_token
        self._ptz_service.Stop(request)

    def _move_camera(self, pan: float, tilt: float):
        request = self._ptz_service.create_type("ContinuousMove")
        request.ProfileToken = self._profile_token
        request.Velocity = {"PanTilt": {"x": pan, "y": tilt}, "Zoom": 0}
        try:
            self._ptz_service.ContinuousMove(request)
        except Exception as e:
            print(f"Error moving camera: {e}")


class PTZCameraNode(Node):
    def __init__(self):
        super().__init__("ptz_camera_node")

        # Load configuration from file
        with open(CONFIG_FILE) as file:
            config = json.load(file)

        # Initialize Camera Control
        self.camera_control = CameraControl(
            config["ip"], config["port"], config["username"], config["password"]
        )

        # Subscriber to the /ptz_cam topic
        self.subscription = self.create_subscription(
            String,
            "/ptz_cam",
            self.listener_callback,
            10,
        )
        self.get_logger().info("PTZ Camera Node is running...")

    def listener_callback(self, msg: String):
        command = msg.data.lower()
        if command in ["up", "down", "left", "right"]:
            self.get_logger().info(f"Moving camera {command}")
            self.camera_control.move_camera(command)
        elif command == "stop":
            self.get_logger().info("Stopping camera")
            self.camera_control.stop_camera()
        else:
            self.get_logger().warn(f"Unknown command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = PTZCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
