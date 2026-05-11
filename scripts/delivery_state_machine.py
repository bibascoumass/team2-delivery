#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String


class DeliveryStateMachine:
    WAIT_FOR_ORDER = "WAIT_FOR_ORDER"
    EXPLORE = "EXPLORE"
    QR_TRACK = "QR_TRACK"
    ARRIVED = "ARRIVED"

    def __init__(self):
        rospy.init_node("delivery_state_machine")

        self.state = self.WAIT_FOR_ORDER
        self.target_qr = None

        self.last_detected_qr = None
        self.last_qr_time = 0.0
        self.qr_offset = None

        self.qr_seen_timeout = rospy.get_param("~qr_seen_timeout", 2.5)
        self.qr_lost_timeout = rospy.get_param("~qr_lost_timeout", 6.0)
        self.arrival_depth = rospy.get_param("~arrival_depth", 0.65)
        self.arrival_area = rospy.get_param("~arrival_area", 0.08)
        self.arrived_hold_time = rospy.get_param("~arrived_hold_time", 2.0)

        self.arrived_since = None

        self.state_pub = rospy.Publisher("/delivery_state", String, queue_size=10, latch=True)

        rospy.Subscriber("/target_qr", String, self.target_cb, queue_size=1)
        rospy.Subscriber("/detected_qr", String, self.detected_qr_cb, queue_size=10)
        rospy.Subscriber("/qr_offset", Point, self.qr_offset_cb, queue_size=10)

        rospy.Timer(rospy.Duration(0.05), self.loop)

        rospy.loginfo("delivery_state_machine started")
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def set_state(self, new_state):
        if self.state == new_state:
            return

        rospy.loginfo("Delivery state: %s -> %s", self.state, new_state)
        self.state = new_state
        self.publish_state()

        if new_state == self.ARRIVED:
            self.arrived_since = rospy.Time.now().to_sec()
        else:
            self.arrived_since = None

    def target_cb(self, msg):
        target = msg.data.strip()
        if not target:
            return

        rospy.loginfo("New order received. Target QR: %s", target)

        self.target_qr = target
        self.last_detected_qr = None
        self.last_qr_time = 0.0
        self.qr_offset = None

        self.set_state(self.EXPLORE)

    def detected_qr_cb(self, msg):
        self.last_detected_qr = msg.data.strip()
        self.last_qr_time = rospy.Time.now().to_sec()

        rospy.loginfo_throttle(
            0.5,
            "QR seen: detected='%s', target='%s', match=%s",
            self.last_detected_qr,
            self.target_qr,
            self.qr_matches_target(self.last_detected_qr, self.target_qr),
        )

    def qr_offset_cb(self, msg):
        self.qr_offset = msg

    def qr_matches_target(self, detected, target):
        if detected is None or target is None:
            return False

        detected = detected.strip().lower()
        target = target.strip().lower()

        if detected == target:
            return True

        return False

    def target_recently_seen(self, timeout):
        if self.target_qr is None:
            return False

        if not self.qr_matches_target(self.last_detected_qr, self.target_qr):
            return False

        return rospy.Time.now().to_sec() - self.last_qr_time <= timeout

    def has_arrived(self):
        if self.qr_offset is None:
            return False

        depth = self.qr_offset.z
        area_frac = self.qr_offset.y

        if depth > 0.0 and depth <= self.arrival_depth:
            return True

        if area_frac >= self.arrival_area:
            return True

        return False

    def reset_order(self):
        rospy.loginfo("Order complete. Returning to WAIT_FOR_ORDER")

        self.target_qr = None
        self.last_detected_qr = None
        self.last_qr_time = 0.0
        self.qr_offset = None

        self.set_state(self.WAIT_FOR_ORDER)

    def loop(self, _event):
        now = rospy.Time.now().to_sec()

        if self.state == self.WAIT_FOR_ORDER:
            return

        if self.state == self.EXPLORE:
            if self.target_recently_seen(self.qr_seen_timeout):
                self.set_state(self.QR_TRACK)
            return

        if self.state == self.QR_TRACK:
            if self.has_arrived():
                self.set_state(self.ARRIVED)
                return

            if not self.target_recently_seen(self.qr_lost_timeout):
                rospy.logwarn("Target QR lost. Returning to EXPLORE")
                self.set_state(self.EXPLORE)

            return

        if self.state == self.ARRIVED:
            if self.arrived_since is not None:
                if now - self.arrived_since >= self.arrived_hold_time:
                    self.reset_order()
            return


if __name__ == "__main__":
    DeliveryStateMachine()
    rospy.spin()
