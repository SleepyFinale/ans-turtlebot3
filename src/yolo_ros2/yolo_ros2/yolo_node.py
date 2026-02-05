import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Load the YOLOv8 model
        self.get_logger().info('YOLO Node has been started.')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Perform object detection
        results = self.model(cv_image, imgsz=320)
        
        # Draw bounding boxes on the image
        annotated_image = results[0].plot()
        
        # Display the annotated image
        cv2.imshow('YOLO Detection', annotated_image)
        cv2.waitKey(1)

        if results and results[0].boxes is not None:
            self.get_logger().info(f"Detected {len(results[0].boxes)} objects")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
