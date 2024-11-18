#!/usr/bin/env python3
'''
This script aims to display detected objects in a real-time camera feed
from the rover.
YOLOv8 and pretrained models are used for object detection
'''

# Import important stuff
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import ReliabilityPolicy, QoSProfile
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO    # YOLOv8

# Import ROS2 Messages
from sensor_msgs.msg import Image

class TargetDetection (Node):
    def __init__(self):
        super().__init__("target_detection")

        # Subscribe
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub_ = self.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", self.image_callback, qos)   # /sensors/camera/color/image

        # Initialize CvBridge (Convert ROS to OpenCV)
        self.bridge = CvBridge()

        # Load the YOLOv8 model
        self.model = YOLO('yolov8_models/yolov8n.pt')     # Path to yolo model folder [yolov8n, yolov8m, yolov8l, yolov8s, yolov8x]

    def image_callback(self, img: Image):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

        # Run YOLOv8 inference
        results = self.model(cv_image)

        # Process and display the results
        for result in results:
            boxes = result.boxes    # Bounding boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
                conf = box.conf[0]  # Get confidence score
                label = result.names[int(box.cls[0])]   # Get class label

                # Draw bounding box and label on the image
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Display the image with detections
        cv2.imshow("YOLOv8 Object Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detect_node = TargetDetection()
    rclpy.spin(detect_node)

    # Clean up
    detect_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


