#!/usr/bin/env python3
'''
This script aims to control a rover to a specified GPS location
Listens to /mors/gps_target topic
Computes X and Y target location
Commands rover directly to target location
11/7/24:
    Currently uses ground truth position from Gazebo simulation
    TO DO: EKF to determine estimated X, Y position
'''

# Import important stuff
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import sqrt, radians, sin, cos, atan2, degrees

# Import ROS2 Messages
from geometry_msgs.msg import Twist, PoseArray      # /cmd_vel
from nav_msgs.msg import Odometry                   # /odom/filtered
from clearpath_gz.msg import GPSTarget              # Custom Message: /mors/gps_target


class Waypoint_Nav_Controller (Node):
    def __init__(self, lat_start, lon_start):
        super().__init__("waypoint_nav_controller")

        # Subscribe
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub_ = self.create_subscription(Odometry, "/a200_0000/platform/odom/filtered", self.odom_callback, qos)   # /platform/odom/filtered
        self.gps_target_sub_ = self.create_subscription(GPSTarget, "/mors/gps_target", self.gps_target_callback, qos)        # /mors/gps_target
        self.true_pose_sub_ = self.create_subscription(PoseArray, "/model/a200_0000/robot/pose", self.true_pose_callback, qos)    # /world/empty_world/dynamic_pose/info

        # Publish 
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/a200_0000/cmd_vel", 10)  # /cmd_vel

        # Initialize variables
        self.lat_start  = lat_start     # Starting latitude
        self.lon_start  = lon_start     # Starting longitude
        self.pose = None
        self.lat_target = self.lon_target = None
        self.x_target = self.lon_target = None

        # Convert target GPS coordinates to meters
        self.GPS = True

        # Timer for loop control
        self.hertz = 10.0                 # 10 Hz Loop Rate
        self.timer = self.create_timer(1/self.hertz, self.control_loop)

        # Controller Parameters
        self.distance_tolerance = 0.0001   # [m]
        self.kp_linear          = 0.5
        self.kp_angular         = 1.0
        self.max_linear_vel     = 1.0   # [m/s]
        self.max_angular_vel    = 1.0   # [m/s]

        # Print node started
        self.get_logger().info("waypoint_navigation node started.")

########## CALLBACK FUNCTIONS ##########
    # Odometry callback to get EKF position (X,Y) and orientation (quaternion) from /a200_0000/platform/odom/filtered topic
    def odom_callback(self, odom: Odometry):
        self.ekf_pose       = odom.pose.pose

    # PoseArray callback to get ground truth pose from /world/empty_world/dynamic_pose/info
    def true_pose_callback(self, truth = PoseArray):
        self.pose = truth.poses[-1]

    # GPS target callback from custom message topic: /mors/gps_target
    def gps_target_callback(self, gps: GPSTarget):
        self.lat_target   = gps.latitude
        self.lon_target   = gps.longitude

        # If GPS=True, convert Lat,Lon to X,Y
        if self.GPS:
            self.x_target, self.y_target = self.compute_goal_pose(self.lat_target, self.lon_target, self.lat_start, self.lon_start)
        else:
            self.x_target = self.lon_target
            self.y_target = self.lat_target
########################################

    def compute_goal_pose(self, lat, lon, lat0, lon0):
        '''
        Calculates goal X and Y position from GPS coordinates
        '''
        # Earth radius in meters
        R = 6371000

        # Convert GPS coordinates to radians
        lat0, lon0 = map(radians, [lat0, lon0])
        lat, lon = map(radians, [lat, lon])

        # Calculate differences
        dlat = lat - lat0
        dlon = lon - lon0

        # Calculate x and y conversion
        x = R * dlon * cos((lat0+lat) / 2)
        y = R * dlat

        return x, y

    def compute_control_command(self):
        '''
        Calculates distance to target and target yaw
        Computes linear and angular command velocities
        '''
        if not self.pose or self.x_target is None or self.y_target is None:
            return Twist()      # No valid pose or target received yet
        
        # Current position and yaw
        self.x0  = self.pose.position.x
        self.y0  = self.pose.position.y
        q   = self.pose.orientation
        self.yaw0 = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        
        # Calculate distance to target
        self.dx = self.x_target - self.x0
        self.dy = self.y_target - self.y0
        self.distance_to_target = sqrt(self.dx**2 + self.dy**2)

        # Stop if close to target (within self.distance_tolerance)
        '''if self.distance_to_target < self.distance_tolerance:
            return Twist()'''

        # Calculate target yaw angle
        self.yaw_target   = atan2(self.dy, self.dx)

        # Calculate current yaw, angle error, and normalize
        dyaw = self.yaw_target - self.yaw0
        self.dyaw = atan2(sin(dyaw), cos(dyaw))       # normalizes yaw to shortest path between [-pi,pi]

        # Controller
        cmd = Twist()
        cmd.linear.x = self.kp_linear * self.distance_to_target
        cmd.angular.z = self.kp_angular * self.dyaw

        # Limit velocities
        cmd.linear.x = min(cmd.linear.x, self.max_linear_vel)   # limits linear velocity to [0,1]
        cmd.angular.z = max(-1.0, min(cmd.angular.z, self.max_angular_vel)) # limits angular velocity to [-1,1]

        #print(cmd)
        print(f"Current Pose: {round(self.x0,2)}, {round(self.y0,2)}, {round(degrees(self.yaw0),2)}")
        print(f"Target Pose:  {round(self.x_target,2)}, {round(self.y_target,2)}, {round(degrees(self.yaw_target),2)}")

        return cmd

    # Computes control command and publishes to /cmd_vel
    def control_loop(self):
        cmd = self.compute_control_command()
        self.cmd_vel_pub_.publish(cmd)


# Main
def main(args=None):
    rclpy.init(args=args)

    # Define start GPS coordinates
    lat_start, lon_start = 40.129355, 18.349940       # TO DO: Obtain from empty_world.sdf file

    waypoint_controller = Waypoint_Nav_Controller(lat_start, lon_start)

    # Spin the node to keep it active
    rclpy.spin(waypoint_controller)

    # Clean up on shutdown
    waypoint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
