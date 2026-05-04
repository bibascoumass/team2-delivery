#!/usr/bin/env python3

import rospy
import csv
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray

class ExploreDebugLogger:
    def __init__(self):
        rospy.init_node("explore_debug_logger")

        self.cmd = Twist()
        self.goal = None
        self.status = -1
        self.map_msg = None

        self.listener = tf.TransformListener()

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_cb)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)

        self.file = open("/root/catkin_ws/src/team2-delivery/explore_debug.csv", "w")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time",
            "robot_x", "robot_y",
            "goal_x", "goal_y",
            "dist_to_goal",
            "cmd_linear_x", "cmd_angular_z",
            "known_ratio",
            "move_base_status"
        ])

    def cmd_cb(self, msg):
        self.cmd = msg

    def goal_cb(self, msg):
        self.goal = msg

    def status_cb(self, msg):
        if msg.status_list:
            self.status = msg.status_list[-1].status

    def map_cb(self, msg):
        self.map_msg = msg

    def known_ratio(self):
        if self.map_msg is None:
            return -1.0
        data = self.map_msg.data
        if not data:
            return -1.0
        known = sum(1 for v in data if v != -1)
        return float(known) / float(len(data))

    def get_robot_pose(self):
        try:
            self.listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(0.2))
            trans, rot = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
            return trans[0], trans[1]
        except Exception:
            return None, None

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rx, ry = self.get_robot_pose()

            gx, gy, dist = None, None, None
            if self.goal is not None and rx is not None:
                gx = self.goal.pose.position.x
                gy = self.goal.pose.position.y
                dist = math.hypot(gx - rx, gy - ry)

            self.writer.writerow([
                rospy.Time.now().to_sec(),
                rx, ry,
                gx, gy,
                dist,
                self.cmd.linear.x,
                self.cmd.angular.z,
                self.known_ratio(),
                self.status
            ])
            self.file.flush()
            rate.sleep()

if __name__ == "__main__":
    node = ExploreDebugLogger()
    node.run()