#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)

        # Publisher to send commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to listen to laser scan data
        self.laser_sub = rospy.Subscriber('/LaserScan', LaserScan, self.laser_callback)

        # Define a safe distance threshold (in meters)
        self.safe_distance = 0.5

        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, data):
        # Process laser scan data
        min_distance = min(data.ranges)
        rospy.loginfo(f"Minimum distance to obstacle: {min_distance}")

        if min_distance < self.safe_distance:
            self.avoid_obstacle()
        else:
            self.move_forward()

    def move_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.5
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)

    def avoid_obstacle(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.5
        self.cmd_vel_pub.publish(move_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
