#!/usr/bin/env python3
import math
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

class NavManager:
    def __init__(self):
        rospy.init_node('nav_manager')

        thresholds = rospy.get_param('/safety_thresholds', {})
        self._warn_dist = thresholds.get('warning_dist', 0.8)
        self._crit_dist = thresholds.get('critical_dist', 0.3)

        self._min_range = float('inf')

        self._safety_pub = rospy.Publisher('/safety_status', SafetyStatus, queue_size=10)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/scan',        LaserScan,   self._scan)
        rospy.Subscriber('/depth_cloud', PointCloud2, self._depth_cloud)
        # rospy.Subscriber('/odom',        Odometry,    self._odom)

        rospy.Timer(rospy.Duration(0.1), self._publish_safety)  # 10 Hz

        rospy.loginfo('nav_manager: ready  warn=%.2f m  crit=%.2f m',
                      self._warn_dist, self._crit_dist)

    def _scan(self, msg):
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        self._min_range = min(valid) if valid else float('inf')

    def _depth_cloud(self, msg):
        pass  # TODO - combine lidar scan data with depth camera point cloud data

    # def _odom(self, msg):
    #     pass  # TODO - (optional?) scale threshold based on robot's velocity (i.e. higher speeds might require a large buffer/braking distance?)

    def _publish_safety(self, _event):
        d = self._min_range
        if d < self._crit_dist:
            level = CRITICAL
        elif d < self._warn_dist:
            level = WARNING
        else:
            level = CLEAR

        status = SafetyStatus()
        status.status_level  = level
        status.sensor_source = 'LiDAR'
        status.distance      = d if math.isfinite(d) else 0.0
        self._safety_pub.publish(status)


def main():
    NavManager()
    rospy.spin()


if __name__ == '__main__':
    main()
