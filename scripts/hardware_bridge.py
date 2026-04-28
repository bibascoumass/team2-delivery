#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SIM_MODE_PARAM = '~sim_mode' # false = stop all motion, true = log errors only
WATCHDOG_TIMEOUT = 0.5  # seconds - ARBITRARY


def cb_cmd_vel(msg, state):
    state['last_cmd_time'] = rospy.Time.now()
    rospy.logdebug('hardware_bridge: /cmd_vel v=%.3f omega=%.3f', msg.linear.x, msg.angular.z)


def main():
    rospy.init_node('hardware_bridge')

    sim_mode = rospy.get_param(SIM_MODE_PARAM, False)
    rospy.loginfo('hardware_bridge: sim_mode=%s', sim_mode)

    state = {'last_cmd_time': rospy.Time.now()}

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)

    rospy.Subscriber('/cmd_vel', Twist, cb_cmd_vel, callback_args=state)

    rate = rospy.Rate(20)  # TODO - ARBITRARY
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - state['last_cmd_time']).to_sec()
        if elapsed > WATCHDOG_TIMEOUT:
            if sim_mode:
                rospy.logwarn_throttle(5.0, 'hardware_bridge: watchdog timeout - motor still running')
            else:
                rospy.logwarn_throttle(5.0, 'hardware_bridge: watchdog timeout — cutting motor power')
                # TODO - send serial stop command to arduino
    
        # TODO - impl
        rate.sleep()


if __name__ == '__main__':
    main()
