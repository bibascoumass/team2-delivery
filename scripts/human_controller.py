#!/usr/bin/env python3
import math
import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState

# ── tunables ─────────────────────────────────────────────────────────────────
WAYPOINTS = [
    (-4.69,  4.89),  # 0  lower-left trash can (start)
    (-4.69,  0.50),  # 1  south — below Wall_101 south end (y=0.85)
    ( 2.55,  0.50),  # 2  east  — below Wall_104 south end; Wall_101 absent here
    ( 2.55,  4.60),  # 3  north — east of Wall_104 (x=2.3)
    ( 1.88,  4.60),  # 4  west  — through Wall_104 door gap y=[4.182, 5.082]
    ( 1.88,  1.91),  # 5  south — kitchen trash can
    ( 1.88,  4.60),  # 6  north — return
    ( 2.55,  4.60),  # 7  east  — exit Wall_104 door
    ( 2.55,  0.50),  # 8  south
    (-4.69,  0.50),  # 9  west  — back to start column
]

MAX_SPEED     = 0.7   # m/s
ARRIVAL_RADIUS = 0.20  # m — advance to next waypoint when this close
DT            = 0.1   # s — matches 10 Hz rate


def make_quaternion_z(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def main():
    rospy.init_node("human_controller")

    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/set_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    rate   = rospy.Rate(1.0 / DT)
    wp_idx = 0

    rospy.loginfo("human_controller: starting kinematic circuit (%d waypoints)",
                  len(WAYPOINTS))

    while not rospy.is_shutdown():
        # ── get ground-truth position ────────────────────────────────────────
        try:
            resp = get_state("human", "world")
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "get_model_state failed: %s", e)
            rate.sleep()
            continue

        px = resp.pose.position.x
        py = resp.pose.position.y

        # check arrival
        tx, ty = WAYPOINTS[wp_idx]
        dx, dy = tx - px, ty - py
        dist   = math.hypot(dx, dy)

        if dist < ARRIVAL_RADIUS:
            wp_idx = (wp_idx + 1) % len(WAYPOINTS)
            tx, ty = WAYPOINTS[wp_idx]
            dx, dy = tx - px, ty - py
            dist   = math.hypot(dx, dy)
            rospy.loginfo("human_controller: waypoint %d  (%.2f, %.2f)",
                          wp_idx, tx, ty)

        if dist < 1e-6:
            rate.sleep()
            continue

        # step to waypoint
        step   = min(MAX_SPEED * DT, dist)
        new_x  = px + (dx / dist) * step
        new_y  = py + (dy / dist) * step
        yaw    = math.atan2(dy, dx)
        qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)

        state = ModelState()
        state.model_name            = "human"
        state.pose.position.x       = new_x
        state.pose.position.y       = new_y
        state.pose.position.z       = 0.0
        state.pose.orientation.x    = 0.0
        state.pose.orientation.y    = 0.0
        state.pose.orientation.z    = qz
        state.pose.orientation.w    = qw
        state.reference_frame       = "world"

        try:
            set_state(state)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, "set_model_state failed: %s", e)

        rate.sleep()


if __name__ == "__main__":
    main()
