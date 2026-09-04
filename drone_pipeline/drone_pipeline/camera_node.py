import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

RTSP_URL = 'rtsp://192.168.144.25:8554/main.264'

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/scout/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open RTSP stream at {RTSP_URL}')
        timer_period = 1.0 / 15.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Camera node started, publishing on /scout/image_raw')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to read frame from RTSP stream')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
