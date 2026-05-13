import sys
import rospy
import rosgraph
import tf
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState
from team2_delivery.msg import ScanTableAction, ScanTableGoal

passed = 0
failed = 0

def check(label, condition, detail=''): # TODO - lookup python equivalent of assert
    global passed, failed
    if condition:
        passed += 1
        print('  [ PASS ]  ' + label)
    else:
        failed += 1
        msg = '  [ FAIL ]  ' + label
        if detail:
            msg += ' — ' + detail
        print(msg)

def topic_set():
    master = rosgraph.Master('/test_stubs')
    pubs, _subs, _svcs = master.getSystemState()
    return {topic for topic, _ in pubs}

def main():
    rospy.init_node('test_stubs', anonymous=True)

    print('\nGazebo integration')
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=5.0)
        check('/gazebo/get_model_state service available', True)
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state('triton', '')
        check('triton model found', resp.success)
    except Exception as e:
        check('/gazebo/get_model_state service available', False, str(e))
        check('triton model found', False, 'service unavailable')

    # position_publisher integration
    print('\n[position_publisher]')
    try:
        rospy.wait_for_message('/odom', Odometry, timeout=5.0)
        check('/odom has live messages', True)
    except Exception as e:
        check('/odom has live messages', False, str(e))

    try:
        listener = tf.TransformListener()
        rospy.sleep(1.0)  # give listener time to fill cache
        listener.waitForTransform('odom', 'base_link', rospy.Time(0), rospy.Duration(5.0))
        listener.lookupTransform('odom', 'base_link', rospy.Time(0)) # TODO - inspect and add to log msg
        check('odom->base_link TF available', True)
    except Exception as e:
        check('odom->base_link TF available', False, str(e))

    topics = topic_set()

    # task_coordinator
    print('\n[task_coordinator]')
    check('/delivery_state advertised', '/delivery_state' in topics)

    tqr_pub = rospy.Publisher('/target_qr', String, queue_size=1)
    rospy.sleep(0.3)
    try:
        tqr_pub.publish(String(data='ORDER_001'))
        check('/target_qr accepted without crash', True) 
    except Exception as e:
        check('/target_qr accepted without crash', False, str(e))

    # nav_manager
    print('\n[nav_manager]')
    check('/safety_status advertised', '/safety_status' in topics)
    check('/cmd_vel advertised', '/cmd_vel' in topics)

    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    rospy.sleep(0.3)
    try:
        scan_pub.publish(LaserScan(
            header=rospy.Header(stamp=rospy.Time.now(), frame_id='base_scan')))
        check('/scan accepted without crash', True)
    except Exception as e:
        check('/scan accepted without crash', False, str(e))

    # perception_hub
    print('\n[perception_hub]')
    check('/qr_data advertised', '/qr_data' in topics)

    scan_client = actionlib.SimpleActionClient('scan_table', ScanTableAction)
    available = scan_client.wait_for_server(timeout=rospy.Duration(5.0))
    check('scan_table action server available', available)

    if available:
        goal = ScanTableGoal(target_order_id='ORDER_001') 
        scan_client.send_goal(goal)
        finished = scan_client.wait_for_result(rospy.Duration(5.0)) # TODO - should check order ID sent == order ID received
        check('ScanTable result received', finished)
        if finished:
            result = scan_client.get_result()
            check('match_found == False (stub behaviour)', not result.match_found)
        else:
            check('match_found == False (stub behaviour)', False, 'no result')
    else:
        check('ScanTable result received', False, 'server unavailable')
        check('match_found == False (stub behaviour)', False, 'server unavailable')

    # hardware_bridge
    print('\n[hardware_bridge]')
    cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(0.3)
    try:
        cmdvel_pub.publish(Twist())
        check('/cmd_vel accepted without crash', True)
    except Exception as e:
        check('/cmd_vel accepted without crash', False, str(e))




    total = passed + failed
    print('\n=== Results: {}/{} passed ===\n'.format(passed, total))
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
