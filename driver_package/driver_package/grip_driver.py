#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class GripDriver(Node):
    def __init__(self):
        super().__init__('grip_driver')
        self.publisher_ = self.create_publisher(Bool, 'status_pwr', 10)
        self.service_ = self.create_service(SetBool, 'grip_power', self.service_callback)
        self.is_power_on = False

    def service_callback(self, request, response):

        if request.data:
            self.is_power_on = True
            self.get_logger().info('Gripper power turned ON')
        else:
            self.is_power_on = False
            self.get_logger().info('Gripper power turned OFF')

        msg = Bool()
        msg.data = self.is_power_on
        self.publisher_.publish(msg)

        response.success = True
        response.message = f'Gripper power is now {"ON" if self.is_power_on else "OFF"}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
