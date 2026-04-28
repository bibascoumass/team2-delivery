#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from team2_delivery.msg import SafetyStatus

# this should fuse /scan and /depth_cloud to publish /safety_status.

# safety levels
CLEAR    = 0
WARNING  = 1
CRITICAL = 2

def cb_scan(msg):
    rospy.logdebug('nav_manager: /scan received — %d ranges', len(msg.ranges))

def cb_depth_cloud(msg):
    rospy.logdebug('nav_manager: /depth_cloud received')

def cb_odom(msg):
    rospy.logdebug('nav_manager: /odom received')

def main():
    rospy.init_node('nav_manager')

    rospy.Subscriber('/scan', LaserScan, cb_scan)
    rospy.Subscriber('/depth_cloud', PointCloud2, cb_depth_cloud)
    rospy.Subscriber('/odom', Odometry, cb_odom)

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    safety_status_pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=10)

    rospy.loginfo('nav_manager: init')
    rospy.spin()


if __name__ == '__main__':
    main()
