#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Twist

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # Subscribers
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/LaserScan', LaserScan, self.laser_callback)
        
        # Publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Variables
        self.imu_data = None
        self.laser_data = None
        self.obstacle_distance_threshold = 0.5  # distance in meters
        self.parking_distance_threshold = 0.4  # distance in meters
        
        self.rate = rospy.Rate(10)
        
    def imu_callback(self, data):
        self.imu_data = data
    
    def laser_callback(self, data):
        self.laser_data = data
    
    def obstacle_avoidance(self):
        if self.laser_data:
            min_distance = min(self.laser_data.ranges)
            rospy.loginfo(f"Minimum distance to obstacle: {min_distance} meters")
            
            if min_distance < self.obstacle_distance_threshold:
                # Obstacle detected within threshold distance
                self.stop_robot()
                self.avoid_obstacle()
            else:
                self.move_forward()
    
    def parking_assistance(self):
        if self.laser_data:
            min_distance = min(self.laser_data.ranges)
            rospy.loginfo(f"Distance to parking spot: {min_distance} meters")
            
            if min_distance < self.parking_distance_threshold:
                self.stop_robot()
                rospy.loginfo("Parking complete.")
    
    def move_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.7
        move_cmd.angular.z = 0.0
        self.cmd_pub.publish(move_cmd)
    
    def stop_robot(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_pub.publish(move_cmd)
    
    def avoid_obstacle(self):
        rospy.loginfo("Avoiding obstacle.")
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.3
        self.cmd_pub.publish(move_cmd)
    
    def run(self):
        while not rospy.is_shutdown():
            self.obstacle_avoidance()
            self.parking_assistance()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
