#!/usr/bin/env python3
'''
This script asks the user for Latitude and Longitude target coordinates
Constantly publishes target coordinates to /mors/gps_target
TO DO: Add starting coordinates to /mors/gps_target message
'''

# Import important stuff
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import ReliabilityPolicy, QoSProfile
import threading

# Import ROS2 Messages
from sensor_msgs.msg import NavSatFix           # /sensors/gps/fix
from clearpath_gz.msg import GPSTarget          # Custom Message: /gps_target


class GPSTargetPublisher (Node):
    def __init__(self):
        super().__init__("gps_target_publisher")

        # Subscribe
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.gps_sub_ = self.create_subscription(NavSatFix, "/a200_0000/sensors/gps_0/fix", self.gps_callback, qos)   # /platform/odom/filtered

        # Publish 
        self.gps_target_pub_ = self.create_publisher(GPSTarget, "/mors/gps_target", 10)  # /mors/gps_target

        # Initialize variables
        self.lat_target = 40.1293568011991           # TO DO: Figure out better way to handle this
        self.lon_target = 18.3499400000000           # TO DO: Figure out better way to handle this
        
        # Timer for loop control
        self.hertz = 1.0                 # 1 Hz Loop Rate
        self.timer = self.create_timer(1/self.hertz, self.publish_coordinates)

        # Start a thread to listen for user input so it doesn't block publishing
        self.input_thread = threading.Thread(target=self.user_input_thread)
        self.input_thread.daemon = True
        self.input_thread.start()

    # GPS callback to get current GPS location from /sensors/gps/fix
    def gps_callback(self, gps: NavSatFix):
        self.lat_current = gps.latitude
        self.lon_current = gps.longitude
        self.alt_current = gps.altitude
    
    # Format and publish message
    def publish_coordinates(self):
        gps_msg = GPSTarget()
        gps_msg.waypoint_number = 1
        gps_msg.latitude = self.lat_target
        gps_msg.longitude = self.lon_target
        self.gps_target_pub_.publish(gps_msg)

    #Asks user for new GPS target in latitude and longitude
    def user_input_thread(self):
        while rclpy.ok():
            try:
                # Ask user for new coordinates
                lat, lon = map(float, input("Enter Target Location (Latitude, Longitude): ").split(","))
                self.lat_target = lat
                self.lon_target = lon
                self.get_logger().info(f"Updated target coordinates to Latitude: {lat}, Longitude: {lon}")
            except ValueError:
                self.get_logger().error("Invalid input. Please enter valid numbers for latitude and longitude.")

def main(args=None):
    rclpy.init(args=args)
    gps_node = GPSTargetPublisher()
    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

