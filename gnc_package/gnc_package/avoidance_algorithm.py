#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class AvoidanceAlgorithm(Node):
    def __init__(self):
        super().__init__('avoidance_algorithm')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(String, 'sensor_data', self.sensor_callback, 10)
        
    def sensor_callback(self, msg):
        pass
    

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceAlgorithm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
