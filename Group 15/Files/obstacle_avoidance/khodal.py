import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class ObstacleAvoidance:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/Lasercan', LaserScan, self.lidar_callback)
        self.imu_sub = rospy.Subscriber('/odom', Odometry, self.imu_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

    def lidar_callback(self, msg):
        # Process lidar data to detect obstacles
        ranges = np.array(msg.ranges)
        angles = np.array(msg.angle_min + msg.angle_increment * np.arange(len(ranges)))
        obstacles = np.where(ranges < 0.5)[0]
        if len(obstacles) > 0:
            # Calculate the direction of the obstacle
            obstacle_angle = angles[obstacles[0]]
            # Send a command to avoid the obstacle
            self.twist.linear.x = 0.1
            self.twist.angular.z = -obstacle_angle
            self.cmd_vel_pub.publish(self.twist)

    def imu_callback(self, msg):
        # Process IMU data to determine the robot's orientation
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        # Use the orientation to adjust the obstacle avoidance behavior

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance')
    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.run()
