#!/usr/bin/env python3
'''
This script aims to track a 'person' target using image feed from camera sensor
YOLOv8 and pretrained models are used for object detection
P controller
11/7/24: 
    Rover rotates in place till target is found, then it follows the target,
    attempting to keep it in the center of the frame and maintain a constant
    distance to the target
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
from sensor_msgs.msg import Image       # /sensors/camera/color/image
from geometry_msgs.msg import Twist     # /cmd_vel

class TargetTracking (Node):
    def __init__(self):
        super().__init__("target_tracking")

        # Subscribe
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub_ = self.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", self.image_callback, qos)   # /sensors/camera/color/image

        # Publish
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/a200_0000/cmd_vel", 10)  # /cmd_vel

        # Initialize Parameters
        self.target_class = 'person'        # Define what to track
        self.center_threshold = 20          # Pixels threshold to consider as "centered"
        self.target_distance = 200          # Desired bounding box height (pixels) for target distance
        self.search_angular_velocity = 0.3  # Angular velocity while searching for a target

        # Initialize Control Parameters
        self.kp_linear = 0.01           # P Gain for linear controller
        self.kp_angular = 0.002         # P Gain for angular controller

        # Initialize CvBridge (Convert ROS to OpenCV)
        self.bridge = CvBridge()

        # Load the YOLOv8 model
        self.model = YOLO('yolov8_models/yolov8n.pt')     # Path to yolo model folder [yolov8n, yolov8m, yolov8l, yolov8s, yolov8x]

    def image_callback(self, img: Image):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        height, width, _ = cv_image.shape
        frame_center = width / 2        # Horizontal center of the frame

        # Run YOLOv8 inference
        results = self.model(cv_image)
        target_detected = False
        target_center_x = 0
        target_height = 0

        # Find the first detection of the target class
        for result in results:
            for box in result.boxes:
                if result.names[int(box.cls[0])] == self.target_class:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get bounding box coordinates
                    target_center_x = (x1 + x2) / 2         # Detected target center
                    target_height = y2 - y1                 
                    target_detected = True

                # Draw bounding box and label on the image
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(cv_image, f"{self.target_class}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                break   # Track only the first instance of the target

        # Initialize Twist message for /cmd_vel
        cmd = Twist()

        # Control logic based on detection
        if target_detected:
            # Horizontal error for angular control
            error_x = frame_center - target_center_x

            # Proportional control for angular velocity
            if abs(error_x) > self.center_threshold:
                cmd.angular.z = self.kp_angular * error_x

            # Vertical error for distance control (based on bounding box height)
            error_distance = self.target_distance - target_height
            cmd.linear.x = self.kp_linear * error_distance

        else:
            # If target is not detected, rotate in place to search
            cmd.angular.z = self.search_angular_velocity

        # Publish Velocity commands
        self.cmd_vel_pub_.publish(cmd)

        # Display the image with detections
        cv2.imshow("YOLOv8 Object Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    track_node = TargetTracking()
    rclpy.spin(track_node)

    # Clean up
    track_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()