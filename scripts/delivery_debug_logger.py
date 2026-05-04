#!/usr/bin/env python3

import csv
import math

import rospy
import tf
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from team2_delivery.msg import (
    QRInfo,
    SafetyStatus,
    ScanTableActionFeedback,
)


STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


SAFETY_NAMES = {
    0: "CLEAR",
    1: "WARNING",
    2: "CRITICAL",
}


def yaw_from_quaternion(q):
    return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class DeliveryDebugLogger:
    def __init__(self):
        rospy.init_node("delivery_debug_logger")

        default_csv = "/root/catkin_ws/src/team2-delivery/delivery_debug.csv"
        self._output_csv = rospy.get_param("~output_csv", default_csv)
        self._rate_hz = rospy.get_param("~rate", 2.0)
        self._near_radius = rospy.get_param("~near_radius", 0.30)
        self._wide_radius = rospy.get_param("~wide_radius", 0.60)

        self._listener = tf.TransformListener()
        self._last_times = {}

        self._delivery_state = ""
        self._target_qr = ""
        self._safety = SafetyStatus(status_level=0, sensor_source="", distance=float("inf"))
        self._qr = QRInfo()
        self._cmd_vel = Twist()
        self._odom = None
        self._amcl_pose = None
        self._scan_min = None
        self._scan_front_min = None
        self._scan_left_min = None
        self._scan_right_min = None
        self._scan_count = 0
        self._move_base_goal = None
        self._move_base_status = -1
        self._move_base_status_text = ""
        self._scan_table_status = -1
        self._scan_table_status_text = ""
        self._scan_table_progress = None
        self._global_plan = Path()
        self._local_plan = Path()
        self._global_costmap = None
        self._local_costmap = None

        rospy.Subscriber("/delivery_state", String, self._delivery_state_cb)
        rospy.Subscriber("/target_qr", String, self._target_qr_cb)
        rospy.Subscriber("/safety_status", SafetyStatus, self._safety_cb)
        rospy.Subscriber("/qr_data", QRInfo, self._qr_cb)
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_cb)
        rospy.Subscriber("/odom", Odometry, self._odom_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_pose_cb)
        rospy.Subscriber("/scan", LaserScan, self._scan_cb)
        rospy.Subscriber("/move_base/current_goal", PoseStamped, self._move_base_goal_cb)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self._move_base_status_cb)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self._global_plan_cb)
        rospy.Subscriber("/move_base/TrajectoryPlannerROS/local_plan", Path, self._local_plan_cb)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self._global_costmap_cb)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self._local_costmap_cb)
        rospy.Subscriber("/scan_table/status", GoalStatusArray, self._scan_table_status_cb)
        rospy.Subscriber("/scan_table/feedback", ScanTableActionFeedback, self._scan_table_feedback_cb)

        self._file = open(self._output_csv, "w")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "time",
            "delivery_state",
            "target_qr",
            "safety_level",
            "safety_name",
            "safety_distance",
            "safety_source",
            "scan_min_range",
            "scan_front_min",
            "scan_left_min",
            "scan_right_min",
            "scan_valid_count",
            "robot_x_map",
            "robot_y_map",
            "robot_yaw_map",
            "amcl_x",
            "amcl_y",
            "amcl_yaw",
            "amcl_xy_error_vs_tf",
            "odom_x",
            "odom_y",
            "odom_yaw",
            "odom_linear_x",
            "odom_angular_z",
            "move_base_status",
            "move_base_status_text",
            "goal_x",
            "goal_y",
            "distance_to_goal",
            "heading_error_to_goal",
            "cmd_mode",
            "cmd_linear_x",
            "cmd_linear_y",
            "cmd_angular_z",
            "tf_map_base_ok",
            "tf_odom_base_ok",
            "tf_base_scan_ok",
            "tf_map_base_error",
            "global_plan_len",
            "local_plan_len",
            "local_plan_end_dist",
            "global_cost_at_robot",
            "global_cost_near_max",
            "global_cost_near_occupied",
            "global_cost_wide_occupied",
            "local_cost_at_robot",
            "local_cost_near_max",
            "local_cost_near_occupied",
            "local_cost_wide_occupied",
            "scan_table_status",
            "scan_table_status_text",
            "scan_table_progress_deg",
            "qr_order_id",
            "qr_detected",
            "qr_match",
            "delivery_state_age",
            "safety_age",
            "cmd_vel_age",
            "scan_age",
            "odom_age",
            "amcl_age",
            "move_base_status_age",
            "move_base_goal_age",
            "global_plan_age",
            "local_plan_age",
            "global_costmap_age",
            "local_costmap_age",
            "scan_table_status_age",
            "qr_age",
        ])
        rospy.loginfo("delivery_debug_logger: writing %s", self._output_csv)

    def _touch(self, topic):
        self._last_times[topic] = rospy.Time.now()

    def _age(self, topic):
        stamp = self._last_times.get(topic)
        if stamp is None:
            return None
        return (rospy.Time.now() - stamp).to_sec()

    def _delivery_state_cb(self, msg):
        self._delivery_state = msg.data
        self._touch("/delivery_state")

    def _target_qr_cb(self, msg):
        self._target_qr = msg.data
        self._touch("/target_qr")

    def _safety_cb(self, msg):
        self._safety = msg
        self._touch("/safety_status")

    def _qr_cb(self, msg):
        self._qr = msg
        self._touch("/qr_data")

    def _cmd_vel_cb(self, msg):
        self._cmd_vel = msg
        self._touch("/cmd_vel")

    def _odom_cb(self, msg):
        self._odom = msg
        self._touch("/odom")

    def _amcl_pose_cb(self, msg):
        self._amcl_pose = msg
        self._touch("/amcl_pose")

    def _scan_cb(self, msg):
        valid = []
        front = []
        left = []
        right = []
        angle = msg.angle_min
        for value in msg.ranges:
            if math.isfinite(value) and value > 0.0:
                valid.append(value)
                if abs(angle) <= math.radians(20.0):
                    front.append(value)
                elif math.radians(60.0) <= angle <= math.radians(120.0):
                    left.append(value)
                elif -math.radians(120.0) <= angle <= -math.radians(60.0):
                    right.append(value)
            angle += msg.angle_increment
        self._scan_min = min(valid) if valid else None
        self._scan_front_min = min(front) if front else None
        self._scan_left_min = min(left) if left else None
        self._scan_right_min = min(right) if right else None
        self._scan_count = len(valid)
        self._touch("/scan")

    def _move_base_goal_cb(self, msg):
        self._move_base_goal = msg
        self._touch("/move_base/current_goal")

    def _move_base_status_cb(self, msg):
        status = self._latest_status(msg)
        if status is not None:
            self._move_base_status = status.status
            self._move_base_status_text = STATUS_NAMES.get(status.status, str(status.status))
        self._touch("/move_base/status")

    def _global_plan_cb(self, msg):
        self._global_plan = msg
        self._touch("/move_base/NavfnROS/plan")

    def _local_plan_cb(self, msg):
        self._local_plan = msg
        self._touch("/move_base/TrajectoryPlannerROS/local_plan")

    def _global_costmap_cb(self, msg):
        self._global_costmap = msg
        self._touch("/move_base/global_costmap/costmap")

    def _local_costmap_cb(self, msg):
        self._local_costmap = msg
        self._touch("/move_base/local_costmap/costmap")

    def _scan_table_status_cb(self, msg):
        status = self._latest_status(msg)
        if status is not None:
            self._scan_table_status = status.status
            self._scan_table_status_text = STATUS_NAMES.get(status.status, str(status.status))
        self._touch("/scan_table/status")

    def _scan_table_feedback_cb(self, msg):
        self._scan_table_progress = msg.feedback.search_progress
        self._touch("/scan_table/feedback")

    def _latest_status(self, msg):
        if not msg.status_list:
            return None
        return msg.status_list[-1]

    def _lookup_pose(self, target, source, timeout=0.03):
        try:
            self._listener.waitForTransform(target, source, rospy.Time(0), rospy.Duration(timeout))
            trans, rot = self._listener.lookupTransform(target, source, rospy.Time(0))
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            return True, trans[0], trans[1], yaw, ""
        except Exception as exc:
            return False, None, None, None, str(exc)

    def _robot_pose_map(self):
        _, x, y, yaw, _ = self._lookup_pose("map", "base_link", timeout=0.05)
        return x, y, yaw

    def _odom_values(self):
        if self._odom is None:
            return None, None, None, None, None
        pose = self._odom.pose.pose
        return (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
            self._odom.twist.twist.linear.x,
            self._odom.twist.twist.angular.z,
        )

    def _amcl_values(self):
        if self._amcl_pose is None:
            return None, None, None
        pose = self._amcl_pose.pose.pose
        return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)

    def _goal_values(self, robot_x, robot_y, robot_yaw):
        if self._move_base_goal is None:
            return None, None, None, None
        goal_x = self._move_base_goal.pose.position.x
        goal_y = self._move_base_goal.pose.position.y
        if robot_x is None or robot_y is None or robot_yaw is None:
            return goal_x, goal_y, None, None
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        heading_error = normalize_angle(math.atan2(dy, dx) - robot_yaw)
        return goal_x, goal_y, math.hypot(dx, dy), heading_error

    def _cmd_mode(self):
        linear = abs(self._cmd_vel.linear.x)
        angular = abs(self._cmd_vel.angular.z)
        if linear < 1e-3 and angular < 1e-3:
            return "stopped"
        if linear < 1e-3:
            return "rotate_only"
        if angular < 1e-3:
            return "forward_only"
        return "arc"

    def _path_end_distance(self, path, robot_x, robot_y):
        if not path.poses or robot_x is None or robot_y is None:
            return None
        pose = path.poses[-1].pose.position
        return math.hypot(pose.x - robot_x, pose.y - robot_y)

    def _costmap_stats(self, costmap, x, y):
        if costmap is None or x is None or y is None:
            return None, None, None, None
        info = costmap.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None, None, None, None

        center_cost = costmap.data[my * info.width + mx]
        near_cells = int(self._near_radius / info.resolution)
        wide_cells = int(self._wide_radius / info.resolution)
        near_max, near_occupied = self._cells_stats(costmap, mx, my, near_cells)
        _, wide_occupied = self._cells_stats(costmap, mx, my, wide_cells)
        return center_cost, near_max, near_occupied, wide_occupied

    def _cells_stats(self, costmap, mx, my, radius_cells):
        info = costmap.info
        max_cost = None
        occupied = 0
        radius_sq = radius_cells * radius_cells
        for y in range(max(0, my - radius_cells), min(info.height, my + radius_cells + 1)):
            for x in range(max(0, mx - radius_cells), min(info.width, mx + radius_cells + 1)):
                if (x - mx) * (x - mx) + (y - my) * (y - my) > radius_sq:
                    continue
                value = costmap.data[y * info.width + x]
                if value >= 50:
                    occupied += 1
                if value >= 0 and (max_cost is None or value > max_cost):
                    max_cost = value
        return max_cost, occupied

    def run(self):
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown():
            map_ok, robot_x, robot_y, robot_yaw, map_error = self._lookup_pose("map", "base_link")
            odom_ok, _, _, _, _ = self._lookup_pose("odom", "base_link")
            scan_tf_ok, _, _, _, _ = self._lookup_pose("base_link", "base_scan")
            odom_x, odom_y, odom_yaw, odom_linear_x, odom_angular_z = self._odom_values()
            amcl_x, amcl_y, amcl_yaw = self._amcl_values()
            goal_x, goal_y, distance_to_goal, heading_error = self._goal_values(robot_x, robot_y, robot_yaw)
            local_plan_end_dist = self._path_end_distance(self._local_plan, robot_x, robot_y)
            global_cost = self._costmap_stats(self._global_costmap, robot_x, robot_y)
            local_cost = self._costmap_stats(self._local_costmap, odom_x, odom_y)

            amcl_error = None
            if None not in (amcl_x, amcl_y, robot_x, robot_y):
                amcl_error = math.hypot(amcl_x - robot_x, amcl_y - robot_y)

            self._writer.writerow([
                rospy.Time.now().to_sec(),
                self._delivery_state,
                self._target_qr,
                self._safety.status_level,
                SAFETY_NAMES.get(self._safety.status_level, str(self._safety.status_level)),
                self._safety.distance,
                self._safety.sensor_source,
                self._scan_min,
                self._scan_front_min,
                self._scan_left_min,
                self._scan_right_min,
                self._scan_count,
                robot_x,
                robot_y,
                robot_yaw,
                amcl_x,
                amcl_y,
                amcl_yaw,
                amcl_error,
                odom_x,
                odom_y,
                odom_yaw,
                odom_linear_x,
                odom_angular_z,
                self._move_base_status,
                self._move_base_status_text,
                goal_x,
                goal_y,
                distance_to_goal,
                heading_error,
                self._cmd_mode(),
                self._cmd_vel.linear.x,
                self._cmd_vel.linear.y,
                self._cmd_vel.angular.z,
                map_ok,
                odom_ok,
                scan_tf_ok,
                "" if map_ok else map_error,
                len(self._global_plan.poses),
                len(self._local_plan.poses),
                local_plan_end_dist,
                global_cost[0],
                global_cost[1],
                global_cost[2],
                global_cost[3],
                local_cost[0],
                local_cost[1],
                local_cost[2],
                local_cost[3],
                self._scan_table_status,
                self._scan_table_status_text,
                self._scan_table_progress,
                self._qr.order_id,
                self._qr.is_detected,
                self._qr.is_match,
                self._age("/delivery_state"),
                self._age("/safety_status"),
                self._age("/cmd_vel"),
                self._age("/scan"),
                self._age("/odom"),
                self._age("/amcl_pose"),
                self._age("/move_base/status"),
                self._age("/move_base/current_goal"),
                self._age("/move_base/NavfnROS/plan"),
                self._age("/move_base/TrajectoryPlannerROS/local_plan"),
                self._age("/move_base/global_costmap/costmap"),
                self._age("/move_base/local_costmap/costmap"),
                self._age("/scan_table/status"),
                self._age("/qr_data"),
            ])
            self._file.flush()
            rate.sleep()


if __name__ == "__main__":
    DeliveryDebugLogger().run()
