#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class ParkingAssistant:
    def __init__(self):
        rospy.init_node('parking_assistant')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)
        self.pose = None
        self.yaw = 0
        self.target_x = 3.0  # Target parking position x
        self.target_y = 3.0  # Target parking position y

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

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        parking_assistant = ParkingAssistant()
        parking_assistant.park_robot()
    except rospy.ROSInterruptException:
        pass




