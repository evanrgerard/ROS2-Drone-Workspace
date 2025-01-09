import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionProcess(Node):
    def __init__(self):
        super().__init__('vision_process')

        self.subscription = self.create_subscription(Image, 'image_raw', self.process_image, 10)
        self.publisher_detected_img = self.create_publisher(Image, 'detected_img', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.bridge = CvBridge()

    def process_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            height, width, _ = cv_image.shape
            cv2.rectangle(cv_image, (width // 4, height // 4), (3 * width // 4, 3 * height // 4), (0, 0, 255), 2)
            
            detected_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher_detected_img.publish(detected_msg)
            self.get_logger().info("Vision Process: Detected object and published modified image.")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcess()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
