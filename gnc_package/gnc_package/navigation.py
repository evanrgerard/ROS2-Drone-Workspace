#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher_ = self.create_publisher(String, 'telem_msg', 10)
        self.client_ = self.create_client(SetBool, 'grip_power')
        
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for grip_power service...")
        self.get_logger().info("grip_power service is now available.")
    
    def cmd_vel_callback(self, msg):
        if msg.linear.x == 0.0:
            self.get_logger().info("Navigation: Drone stopped. Turning grip power ON.")
            self.toggle_grip_power(True)
        else:
            self.get_logger().info("Navigation: Drone moving forward. Turning grip power OFF.")
            self.toggle_grip_power(False)
        
    def toggle_grip_power(self, power_on):
        request = SetBool.Request()
        request.data = power_on

        future = self.client_.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Grip Power Service: {response.message}")
            else:
                self.get_logger().error("Grip Power Service: Request failed")
        except Exception as e:
            self.get_logger().error(f"Grip Power Service: Exception occurred - {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
