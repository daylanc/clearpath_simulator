#!/usr/bin/env python3
'''
This script publishes important data to /mors/data
Makes it easy for post processing
'''

# Import important stuff
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import ReliabilityPolicy, QoSProfile

# Import ROS2 Messages
from geometry_msgs.msg import PoseArray                 # /model/a200_0000/robot/pose
from nav_msgs.msg import Odometry                       # /odom/filtered
from sensor_msgs.msg import NavSatFix                   # /sensors/gps/fix
from clearpath_gz.msg import DataMessage                # Custom Message: /mors/data


class Publish_Data (Node):
    def __init__(self):
        super().__init__("publish_data")

        # Subscribe
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub_ = self.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", self.odom_callback, qos)       # /platform/odom/filtered
        self.gps_sub_ = self.create_subscription(NavSatFix, "/a200_0000/sensors/gps_0/fix", self.gps_callback, qos)             # /sensors/gps/fix
        self.true_pose_sub_ = self.create_subscription(PoseArray, "/model/a200_0000/robot/pose", self.true_pose_callback, qos)  # /model/a200_0000/robot/pose

        # Publish 
        self.data_pub_ = self.create_publisher(DataMessage, "/mors/data", 10) # /mors/data

        # Initialize Data (Errors without this)
        self.ekf_pose = None
        self.true_pose = None
        self.gps_data = None

        # First loop Flag
        self.first_loop = True
        
        # Timer for loop control
        self.hertz = 10.0   # 10 Hz Loop Rate
        self.timer = self.create_timer(1/self.hertz, self.publish_data)

########## CALLBACK FUNCTIONS ##########
    # Odometry callback to get EKF position (X,Y) and orientation (quaternion) from /a200_0000/platform/odom/filtered topic
    def odom_callback(self, odom: Odometry):
        self.ekf_pose = odom.pose.pose

    # PoseArray callback to get ground truth pose from /world/empty_world/dynamic_pose/info
    def true_pose_callback(self, truth = PoseArray):
        self.true_pose = truth.poses[-1]

    # GPS callback to get current GPS location from /sensors/gps/fix
    def gps_callback(self, gps: NavSatFix):
        self.gps_data = gps

########################################
    
    def publish_data(self):
        # If data is available, publish /mors/data topic
        if self.ekf_pose and self.true_pose and self.gps_data:
            
            # If first loop, print node started
            if self.first_loop:
                self.get_logger().info("publish_data node started.")
            
            # Format and publish DataMessage
            data = DataMessage()
            data.x_true   = float(self.true_pose.position.x)
            data.y_true   = float(self.true_pose.position.y)
            data.z_true   = float(self.true_pose.position.z)
            data.qx_true  = float(self.true_pose.orientation.x)
            data.qy_true  = float(self.true_pose.orientation.y)
            data.qz_true  = float(self.true_pose.orientation.z)
            data.qw_true  = float(self.true_pose.orientation.w)
            data.gps_lat  = float(self.gps_data.latitude)
            data.gps_lon  = float(self.gps_data.longitude)
            data.gps_alt  = float(self.gps_data.altitude)
            data.x_ekf    = float(self.ekf_pose.position.x)
            data.y_ekf    = float(self.ekf_pose.position.y)
            data.z_ekf    = float(self.ekf_pose.position.z)
            data.qx_ekf   = float(self.ekf_pose.orientation.x)
            data.qy_ekf   = float(self.ekf_pose.orientation.y)
            data.qz_ekf   = float(self.ekf_pose.orientation.z)
            data.qw_ekf   = float(self.ekf_pose.orientation.w)
            self.data_pub_.publish(data)
            self.first_loop = False

        # If data is not available, wait
        else:
            self.get_logger().warn("Waiting for data on subscribed topics")

# Main
def main(args=None):
    rclpy.init(args=args)

    publish_data_node = Publish_Data()

    # Spin the node to keep it active
    rclpy.spin(publish_data_node)

    # Clean up on shutdown
    publish_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
