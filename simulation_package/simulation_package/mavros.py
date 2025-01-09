import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MavrosNode(Node):
    def __init__(self):
        super().__init__('mavros')
        self.subscription = self.create_subscription(String, 'telem_msg', self.telem_msg, 10)
        
    def telem_msg(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MavrosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
