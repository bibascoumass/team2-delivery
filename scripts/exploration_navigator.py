#!/usr/bin/env python

import yaml
import rospy
import actionlib
import tf.transformations as tft

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class HotspotExplorer:
    def __init__(self):
        rospy.init_node("exploration_navigator")

        self.state = "WAIT_FOR_ORDER"

        self.hotspots_yaml = rospy.get_param("~hotspots_yaml")
        self.frame_id, self.hotspots = self.load_hotspots(self.hotspots_yaml)

        self.goal_timeout_s = rospy.get_param("~goal_timeout_s", 90.0)
        self.scan_time_s = rospy.get_param("~scan_time_s", 2.5)
        self.scan_speed = rospy.get_param("~scan_speed", 0.25)

        self.hotspot_idx = 0
        self.goal_active = False
        self.goal_sent_time = None
        self.scanning_until = 0.0

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("HOTSPOT_EXPLORER: waiting for move_base")
        self.move_base.wait_for_server()
        rospy.loginfo("HOTSPOT_EXPLORER: connected to move_base")

        self.nav_cmd_pub = rospy.Publisher("/nav_cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/delivery_state", String, self.state_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.loop)

        rospy.loginfo("HOTSPOT_EXPLORER: loaded %d hotspots", len(self.hotspots))

    def load_hotspots(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        frame_id = data.get("frame_id", "map")
        hotspots = data.get("hotspots", [])

        if not hotspots:
            raise RuntimeError("No hotspots found in %s" % path)

        return frame_id, hotspots

    def state_cb(self, msg):
        old = self.state
        self.state = msg.data.strip()

        if old != self.state:
            rospy.loginfo("HOTSPOT_EXPLORER: state %s -> %s", old, self.state)

        # Hotspot explorer owns move_base only while state == EXPLORE.
        # When leaving EXPLORE, cancel only this client's active goal and stop any scan twist.
        if old == "EXPLORE" and self.state != "EXPLORE":
            self.stop_explore_behavior()

        # When a new order starts, reset hotspot search.
        # When QR tracking loses the QR and returns to EXPLORE, resume from the current hotspot
        # instead of jumping back to hotspot 0.
        if old != "EXPLORE" and self.state == "EXPLORE":
            if old == "WAIT_FOR_ORDER":
                self.hotspot_idx = 0
            self.goal_active = False
            self.goal_sent_time = None
            self.scanning_until = 0.0
            self.move_base.cancel_goal()
            self.publish_zero()
            rospy.loginfo("HOTSPOT_EXPLORER: ready to explore from idx=%d", self.hotspot_idx)

    def stop_explore_behavior(self):
        self.move_base.cancel_goal()
        self.goal_active = False
        self.goal_sent_time = None
        self.scanning_until = 0.0
        self.publish_zero()

    def loop(self, _event):
        if self.state != "EXPLORE":
            return

        now = rospy.Time.now().to_sec()

        if now < self.scanning_until:
            self.publish_scan()
            return

        if self.scanning_until != 0.0:
            self.scanning_until = 0.0
            self.publish_zero()
            self.hotspot_idx += 1

        if self.goal_active:
            self.handle_active_goal()
            return

        self.send_next_hotspot_goal()

    def handle_active_goal(self):
        status = self.move_base.get_state()

        if self.goal_sent_time is None:
            self.goal_active = False
            return

        age = (rospy.Time.now() - self.goal_sent_time).to_sec()

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("HOTSPOT_EXPLORER: reached hotspot, scanning")
            self.goal_active = False
            self.scanning_until = rospy.Time.now().to_sec() + self.scan_time_s
            return

        if status in [
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.PREEMPTED,
            GoalStatus.LOST,
        ]:
            rospy.logwarn("HOTSPOT_EXPLORER: hotspot goal failed with status %d", status)
            self.goal_active = False
            self.goal_sent_time = None
            self.hotspot_idx += 1
            return

        if age > self.goal_timeout_s:
            rospy.logwarn("HOTSPOT_EXPLORER: hotspot goal timed out")
            self.move_base.cancel_goal()
            self.goal_active = False
            self.goal_sent_time = None
            self.hotspot_idx += 1
            return

    def send_next_hotspot_goal(self):
        hp = self.hotspots[self.hotspot_idx % len(self.hotspots)]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = hp["x"]
        goal.target_pose.pose.position.y = hp["y"]

        yaw = hp.get("yaw", 0.0)
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal)

        self.goal_active = True
        self.goal_sent_time = rospy.Time.now()

        rospy.loginfo(
            "HOTSPOT_EXPLORER: sent %s x=%.2f y=%.2f idx=%d",
            hp.get("name", "hotspot"),
            hp["x"],
            hp["y"],
            self.hotspot_idx,
        )

    def publish_scan(self):
        cmd = Twist()
        cmd.angular.z = self.scan_speed
        self.nav_cmd_pub.publish(cmd)

    def publish_zero(self):
        self.nav_cmd_pub.publish(Twist())


if __name__ == "__main__":
    HotspotExplorer()
    rospy.spin()
