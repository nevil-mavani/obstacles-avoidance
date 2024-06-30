#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)

        # Publisher to the robot's velocity command topic
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to the LIDAR scan topic
        self.scan_sub = rospy.Subscriber('/LaserScan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)
        self.pose = None
        self.yaw = 0
        self.target_x = 1.0  # Target parking position x
        self.target_y = 2.0  # Target parking position y

        
        # Velocity command message
        self.twist = Twist()

        # Parameters
        self.min_distance = 0.5  # Minimum distance to obstacle

        rospy.loginfo("Obstacle avoidance node started")

    def scan_callback(self, data):
        # Process LIDAR data to find the minimum distance
        min_distance = min(data.ranges)

        # Obstacle avoidance logic
        if min_distance < self.min_distance:
            # Obstacle detected, turn the robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5
        else:
            # Path is clear, move forward
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0

        # Publish the velocity command
        self.cmd_pub.publish(self.twist)

    def run(self):
        rospy.spin()

    def odom_callback(self, data):
        self.pose = data.pose.pose
        orientation = self.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    def park_robot(self):
        while not rospy.is_shutdown():
            if self.pose is None:
                continue

            # Compute the error
            error_x = self.target_x - self.pose.position.x
            error_y = self.target_y - self.pose.position.y
            distance_error = math.sqrt(error_x**2 + error_y**2)
            angle_to_goal = math.atan2(error_y, error_x)
            angle_error = angle_to_goal - self.yaw

            # Control logic
            cmd = Twist()
            if distance_error > 0.1:
                cmd.linear.x = 0.2 * distance_error
                cmd.angular.z = 0.5 * angle_error
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                rospy.loginfo("Parked!")
                break

            

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass


