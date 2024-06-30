#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class ParkingAndObstacleAvoidance:
    def __init__(self):
        rospy.init_node('parking_and_obstacle_avoidance_node', anonymous=True)

        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        self.scan_sub = rospy.Subscriber('/LaserScan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odem', Odometry, self.odom_callback)

        # Velocity command message
        self.twist = Twist()

        # Parameters for obstacle avoidance
        self.min_distance = 0.5 # Minimum distance to obstacle

        # Parameters for parking
        self.target_x = 1.0  # Target parking position x
        self.target_y = 1.0  # Target parking position y
        self.pose = None
        self.yaw = 0
        self.rate = rospy.Rate(10)

        rospy.loginfo("Parking and obstacle avoidance node started")

    def scan_callback(self, data):

        min_distance = min(data.ranges)
        # Obstacle avoidance logic
        if min_distance < self.min_distance:
            # Obstacle detected, turn the robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
            print("Obstacle detected",min_distance)
        else:
            # No obstacle detected, reset twist commands
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
            print("Not detected",min_distance)
            # Continue with parking logic
            self.park_robot()

    def odom_callback(self, data):
        self.pose = data.pose.pose
        orientation = self.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def park_robot(self):
        if self.pose is None:
            return

        # Compute the error
        error_x = self.target_x - self.pose.position.x
        error_y = self.target_y - self.pose.position.y
        distance_error = math.sqrt(error_x**2 + error_y**2)
        angle_to_goal = math.atan2(error_y, error_x)
        angle_error = angle_to_goal - self.yaw

        # Control logic
        if distance_error > 0.1:
            self.twist.linear.x = 0.02 * distance_error
            self.twist.angular.z = 0.05 * angle_error
        else:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            rospy.loginfo("Parked!")

    def run(self):
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        parking_and_obstacle_avoidance = ParkingAndObstacleAvoidance()
        parking_and_obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass
