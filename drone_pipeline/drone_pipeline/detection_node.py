import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL_PATH = '/home/varsha/drone_ws/models/best.pt'
CONFIDENCE_THRESHOLD = 0.5

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.get_logger().info(f'Loading YOLO model from {MODEL_PATH}')
        self.model = YOLO(MODEL_PATH)
        self.subscription = self.create_subscription(
            Image, '/scout/image_raw', self.image_callback, 10)
        self.get_logger().info('Detection node started, subscribed to /scout/image_raw')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, verbose=False)[0]

        for box in results.boxes:
            confidence = float(box.conf[0])
            if confidence < CONFIDENCE_THRESHOLD:
                continue
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            self.get_logger().info(
                f'Detected {class_name} at pixel ({center_x:.0f}, {center_y:.0f}) '
                f'confidence={confidence:.2f}'
            )
            # TODO: publish this to geotag_node instead of just logging

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
