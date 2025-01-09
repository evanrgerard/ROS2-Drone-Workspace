import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer


class MJPEGStreamHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            try:
                while True:
                    if self.server.frame is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(self.server.frame)))
                        self.end_headers()
                        self.wfile.write(self.server.frame)
                        self.wfile.write(b"\r\n")
                    else:
                        self.wfile.write(b"--frame\r\n")
            except Exception as e:
                print(f"Client disconnected: {e}")
        else:
            self.send_response(404)
            self.end_headers()


class MJPEGServer(HTTPServer):
    def __init__(self, server_address, RequestHandlerClass):
        super().__init__(server_address, RequestHandlerClass)
        self.frame = None


class GuiStatusNode(Node):
    def __init__(self):
        super().__init__('gui_status')

        self.subscription_image = self.create_subscription(Image, 'detected_img', self.image_callback, 10)
        self.subscription_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.subscription_status = self.create_subscription(String, 'status_pwr', self.status_callback, 10)
        self.subscription_sensor = self.create_subscription(String, 'sensor_data', self.sensor_callback, 10)
        self.bridge = CvBridge()

        self.http_server = MJPEGServer(('0.0.0.0', 8080), MJPEGStreamHandler)
        self.server_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info("HTTP Server started at http://0.0.0.0:8080/stream")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            _, jpeg_frame = cv2.imencode('.jpg', cv_image)

            self.http_server.frame = jpeg_frame.tobytes()
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            
    def cmd_callback(self, msg):
        pass
    
    def status_callback(self, msg):
        pass
    
    def sensor_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GuiStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
