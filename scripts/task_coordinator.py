#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty, SetBool
from move_base_msgs.msg import MoveBaseAction
from team2_delivery.msg import QRInfo, SafetyStatus


def cb_target_qr(msg):
    rospy.logdebug('task_coordinator: /target_qr received: %s', msg.data)

def cb_qr_data(msg):
    rospy.logdebug('task_coordinator: /qr_data — detected=%s match=%s', msg.is_detected, msg.is_match)

def cb_safety_status(msg):
    rospy.logdebug('task_coordinator: /safety_status — level=%d source=%s dist=%.2f',
                   msg.status_level, msg.sensor_source, msg.distance)

def main():
    rospy.init_node('task_coordinator')

    delivery_state_pub = rospy.Publisher('/delivery_state', String, queue_size=10)

    rospy.Subscriber('/target_qr', String, cb_target_qr)
    rospy.Subscriber('/qr_data', QRInfo, cb_qr_data)
    rospy.Subscriber('/safety_status', SafetyStatus, cb_safety_status)

    # move_base action client
    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    rospy.loginfo('task_coordinator: waiting for /move_base action server...')
    if not move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logwarn('task_coordinator: /move_base not available — navigation disabled (Phase 0 stub)')

    # TODO:
    # /save_map  
    # /reset_localization 
    # /confirm_delivery

    rospy.loginfo('task_coordinator init')
    rospy.spin()


if __name__ == '__main__':
    main()
