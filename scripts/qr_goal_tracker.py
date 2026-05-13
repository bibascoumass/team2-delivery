#!/usr/bin/env python

import math
import actionlib
import rospy
import tf.transformations as tft
import tf2_geometry_msgs
import tf2_ros

from geometry_msgs.msg import Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_srvs.srv import Empty


class QRGoalTracker:
    def __init__(self):
        rospy.init_node("qr_goal_tracker")

        self.state = "WAIT_FOR_ORDER"
        self.qr_offset = None
        self.last_qr_time = 0.0
        self.last_goal_time = 0.0
        self.last_sent_xy = None

        
        self.qr_timeout = rospy.get_param("~qr_timeout", 2.0)

        
        self.goal_update_period = rospy.get_param("~goal_update_period", 0.8)

        
        self.goal_move_threshold = rospy.get_param("~goal_move_threshold", 0.20)

      
        self.arrival_depth = rospy.get_param("~arrival_depth", 0.35)

        
        self.min_goal_forward = rospy.get_param("~min_goal_forward", 0.12)
        self.max_goal_forward = rospy.get_param("~max_goal_forward", 2.5)

       
        self.camera_fov_x = rospy.get_param("~camera_fov_x", 1.047)

        rospy.Subscriber("/delivery_state", String, self.state_cb, queue_size=1)
        rospy.Subscriber("/qr_offset", Point, self.qr_offset_cb, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.clear_costmaps_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        rospy.loginfo("qr_goal_tracker waiting for move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("qr_goal_tracker connected to move_base")

        rospy.Timer(rospy.Duration(0.05), self.loop)

    def state_cb(self, msg):
        old_state = self.state
        self.state = msg.data.strip()

        if old_state == "QR_TRACK" and self.state != "QR_TRACK":
            rospy.loginfo("QR_TRACK: leaving QR_TRACK, canceling QR goal")
            self.move_base.cancel_goal()
            self.last_sent_xy = None

        if old_state != "QR_TRACK" and self.state == "QR_TRACK":
            rospy.loginfo("QR_TRACK: entering QR_TRACK")
            self.last_goal_time = 0.0
            self.last_sent_xy = None

            try:
                self.clear_costmaps_srv()
            except Exception as e:
                rospy.logwarn_throttle(2.0, "Could not clear costmaps: %s", e)

    def qr_offset_cb(self, msg):
        self.qr_offset = msg
        self.last_qr_time = rospy.Time.now().to_sec()

    def qr_data_fresh(self):
        if self.qr_offset is None:
            return False

        age = rospy.Time.now().to_sec() - self.last_qr_time
        return age <= self.qr_timeout

    def make_goal_from_qr(self):
        if self.qr_offset is None:
            return None

        x_error = self.qr_offset.x
        depth = self.qr_offset.z

        if depth <= 0.0:
            rospy.logwarn_throttle(1.0, "QR_TRACK: invalid depth %.3f", depth)
            return None

      
        forward_dist = depth - self.arrival_depth
        forward_dist = max(
            self.min_goal_forward,
            min(forward_dist, self.max_goal_forward)
        )

       
        lateral_dist = -x_error * depth * math.tan(self.camera_fov_x / 2.0)

        goal_base = PoseStamped()
        goal_base.header.frame_id = "base_link"
        goal_base.header.stamp = rospy.Time(0)

        goal_base.pose.position.x = forward_dist
        goal_base.pose.position.y = lateral_dist
        goal_base.pose.position.z = 0.0

      
        goal_base.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rospy.Time(0),
                rospy.Duration(0.2)
            )
            goal_map = tf2_geometry_msgs.do_transform_pose(goal_base, transform)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "QR goal transform failed: %s", e)
            return None

        gx = goal_map.pose.position.x
        gy = goal_map.pose.position.y

        bx = transform.transform.translation.x
        by = transform.transform.translation.y

        
        yaw = math.atan2(gy - by, gx - bx)
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo_throttle(
            1.0,
            "QR_TRACK: depth=%.2f x_error=%.2f forward=%.2f lateral=%.2f goal=(%.2f, %.2f)",
            depth,
            x_error,
            forward_dist,
            lateral_dist,
            gx,
            gy,
        )

        return goal

    def should_send_goal(self, goal):
        gx = goal.target_pose.pose.position.x
        gy = goal.target_pose.pose.position.y

        if self.last_sent_xy is None:
            return True

        lx, ly = self.last_sent_xy
        dist = math.hypot(gx - lx, gy - ly)

        return dist >= self.goal_move_threshold

    def loop(self, _event):
        if self.state != "QR_TRACK":
            return

        if not self.qr_data_fresh():
            rospy.logwarn_throttle(1.0, "QR_TRACK: waiting for fresh QR data")
            return

        now = rospy.Time.now().to_sec()

        if now - self.last_goal_time < self.goal_update_period:
            return

        goal = self.make_goal_from_qr()
        if goal is None:
            return

        if not self.should_send_goal(goal):
            return

        self.move_base.send_goal(goal)

        self.last_goal_time = now
        self.last_sent_xy = (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
        )

        rospy.loginfo(
            "QR_TRACK: sent QR move_base goal x=%.2f y=%.2f",
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
        )


if __name__ == "__main__":
    QRGoalTracker()
    rospy.spin()
