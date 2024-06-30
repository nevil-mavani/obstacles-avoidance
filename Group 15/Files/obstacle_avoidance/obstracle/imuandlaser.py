#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        
        self.scan_sub = rospy.Subscriber('/LaserScan', LaserScan, self.scan_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.rate = rospy.Rate(10)
        
        self.scan_data = None
        self.imu_data = None
        
    def scan_callback(self, data):
        self.scan_data = data
    
    def imu_callback(self, data):
        self.imu_data = data
    
    def obstacle_avoidance(self):
        while not rospy.is_shutdown():
            if self.scan_data is None or self.imu_data is None:
                continue
            
            scan_ranges = np.array(self.scan_data.ranges)
            min_distance = np.min(scan_ranges)
            
            twist = Twist()
            
            if min_distance < 0.5:  # If an obstacle is closer than 1 meter
                twist.linear.x = 0.0
                twist.angular.z = 0.5
            else:
                twist.linear.x = 0.5
                twist.angular.z = 0.0
            
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        node.obstacle_avoidance()
    except rospy.ROSInterruptException:
        pass
