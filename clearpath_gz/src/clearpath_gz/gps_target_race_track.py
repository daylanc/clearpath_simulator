#!/usr/bin/env python3
'''
This script imports a CSV file of waypoints
Constantly publishes target coordinates to /mors/gps_target
When current target is reached, scripts publishes next target in list
Last waypoint will be constantly published till script is terminated
'''

# Import important stuff
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import ReliabilityPolicy, QoSProfile
from math import sqrt, radians, sin, cos, atan2
import pandas as pd

# Import ROS2 Messages
from sensor_msgs.msg import NavSatFix           # /sensors/gps/fix
from clearpath_gz.msg import GPSTarget          # Custom Message: /gps_target


class GPSTargetPublisher (Node):
    def __init__(self, waypoints_file):
        super().__init__("gps_target_publisher")

        # Subscribe
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.gps_sub_ = self.create_subscription(NavSatFix, "/a200_0000/sensors/gps_0/fix", self.gps_callback, qos)   # /platform/odom/filtered

        # Publish 
        self.gps_target_pub_ = self.create_publisher(GPSTarget, "/mors/gps_target", 10)  # /mors/gps_target

        # Lodad waypoints data as dataframe
        self.waypoints_df = pd.read_csv(waypoints_file)
        self.current_waypoint_index = 0
        
        # Initialize variables
        self.dist_tolerance = 3.0       # [m]
        self.current_lat = None
        self.current_lon = None
        self.done = False
        
        # Timer for loop control
        self.hertz = 1.0                 # 1 Hz Loop Rate
        self.timer = self.create_timer(1/self.hertz, self.publish_coordinates)

        # Start by publishing first waypoint
        self.publish_coordinates()

    # GPS callback to get current GPS location from /sensors/gps/fix
    def gps_callback(self, gps: NavSatFix):
        self.current_lat = gps.latitude
        self.current_lon = gps.longitude

    def publish_coordinates(self):
        '''
        Determines the next waypoint in the CSV file, repeat last waypoint if last
        Creates and publishes the GPS target message to /mors/gps_target
        Calculates current distance to next target. If within range, publish next waypoint
        '''

        # If last waypoint, keep publishing the last one
        if self.current_waypoint_index >= len(self.waypoints_df):
            target_waypoint = self.waypoints_df.iloc[-1]
            self.done = True
        else:
            target_waypoint = self.waypoints_df.iloc[self.current_waypoint_index]
        
        # Format and publish message
        gps_msg = GPSTarget()
        gps_msg.waypoint_number = int(target_waypoint["Waypoint"])
        gps_msg.latitude = target_waypoint["Latitude"]
        gps_msg.longitude = target_waypoint["Longitude"]
        self.gps_target_pub_.publish(gps_msg)

        # Check if close enough to the current waypoint to move to the next one
        if self.current_lat is not None and self.current_lon is not None:
            distance = self.calculate_distance(self.current_lat, self.current_lon, target_waypoint["Latitude"], target_waypoint["Longitude"])
            if distance < self.dist_tolerance and not self.done:
                self.get_logger().info(f"Reached Waypoint {int(target_waypoint['Waypoint'])}. Moving to next.")
                self.current_waypoint_index += 1
            if self.done:
                self.get_logger().info("End of Mission.")

    def calculate_distance(self, lat0, lon0, lat1, lon1):
        '''
        Calculate distance between two GPS coordinates in meters
        Used for determining when the next GPS waypoint will be published
        '''
        R = 6371000     # Radius of Earth in [m]
        phi0 = radians(lat0)
        phi1 = radians(lat1)
        delta_phi = radians(lat1 - lat0)
        delta_lambda = radians(lon1 - lon0)

        a = sin(delta_phi/2)**2 + cos(phi0) * cos(phi1) * sin(delta_lambda/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        return R * c

def main(args=None):
    rclpy.init(args=args)

    # Path to waypoints CSV file
    waypoints_file = "/home/mors/clearpath_ws/src/clearpath_simulator/clearpath_gz/src/clearpath_gz/waypoints/gps_waypoints.csv"

    gps_node = GPSTargetPublisher(waypoints_file)

    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

