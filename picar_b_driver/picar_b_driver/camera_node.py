#!/usr/bin/env python3
"""ROS2 camera node for Raspberry Pi Camera (libcamera/picamera2).

Publishes Image and CameraInfo messages from the Pi Camera.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from picamera2 import Picamera2


class CameraNode(Node):
    def __init__(self):
        super().__init__('picar_camera')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', 'camera_link')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Initialise picamera2
        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()

        self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(
            f'Camera node started: {self.width}x{self.height} @ {self.fps} Hz'
        )

    def publish_frame(self):
        frame = self.camera.capture_array()
        stamp = self.get_clock().now().to_msg()

        # Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        # CameraInfo (basic — no calibration)
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.width
        info_msg.height = self.height
        self.info_pub.publish(info_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down camera node')
        self.camera.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
