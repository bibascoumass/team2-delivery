#!/usr/bin/env python3

import csv
import math
import os
from collections import deque

import rospy
import tf
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rosgraph_msgs.msg import Log


LOG_NODE_FILTERS = ("move_base", "explore", "slam_gmapping")
DIST_WINDOW_SEC = 5.0


class ExploreDebugLogger:
    def __init__(self):
        rospy.init_node("explore_debug_logger")

        self.cmd = Twist()
        self.goal = None
        self.status = -1
        self.map_msg = None
        self.global_plan_len = 0
        self.goal_xy = None
        self.goal_change_time = None
        self.pose_history = deque()

        self.listener = tf.TransformListener()

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_cb)
        rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_cb)
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.plan_cb)
        rospy.Subscriber("/rosout_agg", Log, self.rosout_cb)

        default_csv = "/tmp/explore_debug.csv"
        output_csv = rospy.get_param("~output_csv", default_csv)
        rospy.loginfo("explore_debug_logger: writing %s", output_csv)
        self.file = open(output_csv, "w")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time",
            "robot_x", "robot_y",
            "goal_x", "goal_y",
            "dist_to_goal",
            "cmd_linear_x", "cmd_angular_z",
            "known_ratio",
            "move_base_status",
            "global_plan_len",
            "time_on_current_goal",
            "dist_5s",
        ])

        log_path = os.path.splitext(output_csv)[0] + ".log"
        rospy.loginfo("explore_debug_logger: writing %s", log_path)
        self.log_file = open(log_path, "w")

    def cmd_cb(self, msg):
        self.cmd = msg

    def goal_cb(self, msg):
        new_xy = (msg.pose.position.x, msg.pose.position.y)
        if self.goal_xy != new_xy:
            self.goal_xy = new_xy
            self.goal_change_time = rospy.Time.now()
        self.goal = msg

    def status_cb(self, msg):
        if msg.status_list:
            self.status = msg.status_list[-1].status

    def map_cb(self, msg):
        self.map_msg = msg

    def plan_cb(self, msg):
        self.global_plan_len = len(msg.poses)

    def rosout_cb(self, msg):
        if not any(f in msg.name for f in LOG_NODE_FILTERS):
            return
        if msg.level < Log.INFO:
            return
        level_name = {Log.DEBUG: "DEBUG", Log.INFO: "INFO", Log.WARN: "WARN",
                      Log.ERROR: "ERROR", Log.FATAL: "FATAL"}.get(msg.level, "?")
        line = "%.3f %-5s %s: %s\n" % (
            msg.header.stamp.to_sec(), level_name, msg.name, msg.msg)
        self.log_file.write(line)
        self.log_file.flush()

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

    def update_dist_window(self, now_sec, rx, ry):
        if rx is None:
            return None
        self.pose_history.append((now_sec, rx, ry))
        cutoff = now_sec - DIST_WINDOW_SEC
        while self.pose_history and self.pose_history[0][0] < cutoff:
            self.pose_history.popleft()
        if len(self.pose_history) < 2:
            return 0.0
        ox, oy = self.pose_history[0][1], self.pose_history[0][2]
        return math.hypot(rx - ox, ry - oy)

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rx, ry = self.get_robot_pose()

            gx, gy, dist = None, None, None
            if self.goal is not None and rx is not None:
                gx = self.goal.pose.position.x
                gy = self.goal.pose.position.y
                dist = math.hypot(gx - rx, gy - ry)

            now = rospy.Time.now()
            now_sec = now.to_sec()
            tog = None
            if self.goal_change_time is not None:
                tog = (now - self.goal_change_time).to_sec()

            dist5 = self.update_dist_window(now_sec, rx, ry)

            self.writer.writerow([
                now_sec,
                rx, ry,
                gx, gy,
                dist,
                self.cmd.linear.x,
                self.cmd.angular.z,
                self.known_ratio(),
                self.status,
                self.global_plan_len,
                tog,
                dist5,
            ])
            self.file.flush()
            rate.sleep()


if __name__ == "__main__":
    node = ExploreDebugLogger()
    node.run()
