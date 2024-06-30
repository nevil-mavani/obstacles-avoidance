#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math

class ParkingAssistant:
    def __init__(self):
        rospy.init_node('parking_assistant', anonymous=True)

        self.target_position = [1.0, 1.0]  # Target parking position (x, y)
        self.tolerance = 0.1  # Tolerance for parking
        self.max_speed = 0.4  # Maximum speed

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_position = [0.0, 0.0]
        self.current_orientation = 0.0

    def odom_callback(self, data):
        self.current_position = [data.pose.pose.position.x, data.pose.pose.position.y]
        self.current_orientation = self.get_yaw_from_quaternion(data.pose.pose.orientation)

    def get_yaw_from_quaternion(self, orientation):
        # Convert quaternion to yaw
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return euler[2]

    def move_to_target(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            distance = self.calculate_distance(self.current_position, self.target_position)
            if distance > self.tolerance:
                angle_to_target = math.atan2(self.target_position[1] - self.current_position[1],
                                             self.target_position[0] - self.current_position[0])
                angle_diff = angle_to_target - self.current_orientation

                # Create a Twist message and publish it
                twist = Twist()
                twist.linear.x = min(self.max_speed, distance)
                twist.angular.z = angle_diff
                self.cmd_vel_pub.publish(twist)
            else:
                rospy.loginfo("Arrived at the parking position.")
                self.max_speed = 0
                break
            rate.sleep()

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

if __name__ == '__main__':
    try:
        assistant = ParkingAssistant()
        assistant.move_to_target()
    except rospy.ROSInterruptException:
        pass
