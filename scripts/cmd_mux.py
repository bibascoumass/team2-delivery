#!/usr/bin/env python

"""
  WAIT_FOR_ORDER -> stop
  EXPLORE        -> forward /nav_cmd_vel
  QR_TRACK       -> forward /nav_cmd_vel
  ARRIVED        -> stop
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CmdMux:
    def __init__(self):
        rospy.init_node("cmd_mux")

        self.state = "WAIT_FOR_ORDER"
        self.nav_cmd = Twist()
        self.last_nav_time = 0.0

        self.timeout = rospy.get_param("~timeout", 0.4)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/delivery_state", String, self.state_cb, queue_size=1)
        rospy.Subscriber("/nav_cmd_vel", Twist, self.nav_cb, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.loop)

        rospy.loginfo("cmd_mux started")

    def state_cb(self, msg):
        self.state = msg.data.strip()

    def nav_cb(self, msg):
        self.nav_cmd = msg
        self.last_nav_time = rospy.Time.now().to_sec()

    def loop(self, _event):
        cmd = Twist()
        fresh = rospy.Time.now().to_sec() - self.last_nav_time <= self.timeout

        if self.state in ["EXPLORE", "QR_TRACK"] and fresh:
            cmd = self.nav_cmd

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    CmdMux()
    rospy.spin()
