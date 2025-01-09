#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        image_path = '/home/re/drone_ros2_workspace/src/doc/images.jpeg'
        self.image = cv2.imread(image_path)

        if self.image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
        else:
            self.get_logger().info(f"Image loaded successfully from: {image_path}")

        self.bridge = CvBridge()

    def timer_callback(self):
        if self.image is not None:
            image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
            self.publisher_.publish(image_msg)
            self.get_logger().info("Publishing image")
        else:
            self.get_logger().error("No image to publish")


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
