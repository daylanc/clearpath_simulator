#!/usr/bin/env python3
'''
This script aims to follow a path using RGB camera
'''

# Import important stuff
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import ReliabilityPolicy, QoSProfile
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO    # YOLOv8

# Import ROS2 Messages
from sensor_msgs.msg import Image       # /sensors/camera/color/image
from geometry_msgs.msg import Twist     # /cmd_vel

class PathFollowing (Node):
    def __init__(self):
        super().__init__("path_following")

        # Subscribe
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub_ = self.create_subscription(Image, "/a200_0000/sensors/camera_0/color/image", self.image_callback, qos)   # /sensors/camera/color/image

        # Publish
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/a200_0000/cmd_vel", 10)  # /cmd_vel

        # Initialize Control Parameters
        self.kp_angular = 0.005         # P Gain for angular controller

        # Initialize CvBridge (Convert ROS to OpenCV)
        self.bridge = CvBridge()

    def image_callback(self, img: Image):
        '''
        Converts ROS image to open_cv image. Calls process_image function to find the road. Calls control_rover to update /cmd_vel
        '''
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

        # Process image to find the road
        processed_frame = self.process_image(cv_image)

        # Generate control commands based on road position
        self.control_rover(processed_frame, cv_image)

    def process_image(self, frame):
        '''
        Uses Canny edge detection to calculate the center of the road in the image frame
        '''
        # Crop the image to the lower part of the frame
        height, width = frame.shape[:2]
        cropped_frame = frame[int(height*0.6):height, :]    # Use only bottom 40% of image
        
        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        # Perform edge detection
        edges = cv2.Canny(blur, 50, 150)

        # Mask to focus on lower part of image (typically where the road appears)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[(0,height), (width,height), (width,int(height*0.5)), (0,int(height*0.5))]])
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Detect lines through Hough transform
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=150)

        # Draw Lines and calculate the center of the road
        road_center = width // 2
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1,y1+int(height*0.6)), (x2,y2+int(height*0.6)), (0,255,0), 3)
                road_center = (x1+x2) // 2  # Approximate center
        
        # Show image
        cv2.circle(frame, (road_center, height-30), 5, (0,0,255), -1)   # Mark center
        cv2.imshow('Road Detection', frame)
        cv2.waitKey(1)

        return road_center

    def control_rover(self, road_center, frame):
        '''
        Calculates the error between frame center and road_center. Controls rover to stay in middle of road
        '''
        # Calculate frame center
        frame_center = frame.shape[1] // 2

        # Calculate error between frame_center and road_center
        error = frame_center - road_center

        # Format /cmd_vel message
        cmd = Twist()
        cmd.linear.x = 0.2                          # Forward speed
        cmd.angular.z = self.kp_angular * error     # Angular speed
        self.cmd_vel_pub_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    follow_node = PathFollowing()
    rclpy.spin(follow_node)

    # Clean up
    follow_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()