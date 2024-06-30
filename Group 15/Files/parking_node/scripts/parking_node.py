#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ParkingNode:
    def __init__(self):
        rospy.init_node('parking_node', anonymous=True)

        # Publisher to send commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber to listen for a parking trigger (e.g., from a sensor or a topic)
        self.parking_sub = rospy.Subscriber('/parking_trigger', Bool, self.parking_callback)

        # Define parking position and orientation
        self.parking_position = (5.0, 5.0)  # Example parking coordinates (x, y)
        self.parking_orientation = 0.0      # Example orientation (yaw angle)

        self.rate = rospy.Rate(10)  # 10 Hz

    def parking_callback(self, data):
        if data.data:
            rospy.loginfo("Parking triggered!")
            self.park_robot()

    def park_robot(self):
        rospy.loginfo("Starting parking maneuver...")

        # Example maneuver: Move forward, turn, and stop

        move_cmd = Twist()
        
        # Move forward
        move_cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(2)  # Move forward for 2 seconds
        
        # Stop
        move_cmd.linear.x = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)
        
        # Rotate (simulate parking maneuver)
        move_cmd.angular.z = 1.0
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(2)  # Rotate for 2 seconds

        # Stop
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(1)

        rospy.loginfo("Parking maneuver completed.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ParkingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
