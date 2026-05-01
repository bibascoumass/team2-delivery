#!/usr/bin/env python3
import math
import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from team2_delivery.msg import QRInfo, SafetyStatus, ScanTableAction, ScanTableGoal

# safety levels
CLEAR    = 0
WARNING  = 1
CRITICAL = 2

# states
WAIT_FOR_ORDER = 'WAIT_FOR_ORDER' # waits for /target_qr
HOTSPOT_SEARCH_LOOP = 'HOTSPOT_SEARCH_LOOP' # navigates to each table in sequence via move_base
DOCK_AND_DELIVER = 'DOCK_AND_DELIVER' # waits dock_wait_secs for delivery confirmation
NAV_TO_HOME = 'NAV_TO_HOME' 
AVOID_DYNAMIC = 'AVOID_DYNAMIC' # pauses on WARNING safety alert; resumes previous state on CLEAR
ESTOP = 'ESTOP' # hard stop on CRITICAL safety alert

DELIVERY_STATES = {HOTSPOT_SEARCH_LOOP, DOCK_AND_DELIVER, NAV_TO_HOME}

def _yaw_to_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))

class TaskCoordinator:
    def __init__(self):
        rospy.init_node('task_coordinator')

        tables_raw = rospy.get_param('/table_list', [])
        self._use_hotspots = rospy.get_param('~use_hotspots', False)
        if self._use_hotspots:
            self._tables = self._load_hotspots()
            rospy.loginfo('task_coordinator: hotspot mode — %d hotspots loaded',
                          len(self._tables))
        else:
            self._tables = tables_raw  # list of {id, x, y, theta}

        settings = rospy.get_param('/delivery_settings', {})
        self._dock_wait   = settings.get('dock_wait_secs', 5.0)
        self._kitchen     = {'x': 1.5, 'y': 1.8, 'theta': 0.0}

        self._state = WAIT_FOR_ORDER
        self._state_stack = [] # interrupt resume stack (AVOID_DYNAMIC / ESTOP)
        self._order_id = None
        self._table_idx = 0
        self._qr_match = False
        self._safety_level = CLEAR
        self._dock_start = None
        self._scanning = False

        self._state_pub = rospy.Publisher('/delivery_state', String,
                                            queue_size=10, latch=True)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self._mb = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._mb_available = self._mb.wait_for_server(timeout=rospy.Duration(3.0))
        if self._mb_available:
            rospy.loginfo('task_coordinator: /move_base available')
        else:
            rospy.logwarn('task_coordinator: /move_base not available — navigation disabled')

        # ScanTable client
        self._scan_client = actionlib.SimpleActionClient('scan_table', ScanTableAction)
        self._scan_available = self._scan_client.wait_for_server(timeout=rospy.Duration(3.0))    

        rospy.Subscriber('/target_qr', String, self._target_qr)
        rospy.Subscriber('/qr_data', QRInfo, self._qr_data)
        rospy.Subscriber('/safety_status', SafetyStatus, self._safety_status)

        rospy.Timer(rospy.Duration(0.05), self._loop)  # 20 Hz

        self._publish_state()
        rospy.loginfo('task_coordinator: ready (%d tables loaded)', len(self._tables))

    def _load_hotspots(self):
        import yaml, os
        path = rospy.get_param(
            '~hotspots_yaml',
            os.path.join(os.path.dirname(__file__), '..', 'config', 'hotspots.yaml'))
        try:
            with open(path) as f:
                data = yaml.safe_load(f)
            return [{'id': h['name'], 'x': h['x'], 'y': h['y'], 'theta': h['yaw']}
                    for h in data.get('hotspots', [])]
        except Exception as e:
            rospy.logwarn('task_coordinator: could not load hotspots (%s) — falling back to table_list', e)
            return rospy.get_param('/table_list', [])

    def _target_qr(self, msg):
        order = msg.data.strip()
        if not order:
            return
        rospy.loginfo('task_coordinator: order received — %s', order)
        self._cancel_scan()
        self._state_stack.clear()
        self._order_id  = order
        self._table_idx = 0
        self._qr_match  = False
        self._set_state(HOTSPOT_SEARCH_LOOP)

    def _qr_data(self, msg):
        self._qr_match = msg.is_match

    def _safety_status(self, msg):
        self._safety_level = msg.status_level

    def _publish_state(self):
        self._state_pub.publish(String(data=self._state))

    def _set_state(self, new_state):
        if self._state == new_state:
            return
        rospy.loginfo('task_coordinator: %s → %s', self._state, new_state)
        self._state = new_state
        self._publish_state()

    def _send_nav_goal(self, x, y, theta):
        if not self._mb_available:
            return False
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = _yaw_to_quaternion(theta)
        self._mb.send_goal(goal)
        return True

    def _cancel_nav(self):
        if self._mb_available:
            self._mb.cancel_all_goals()

    def _cancel_scan(self):
        if self._scan_available and self._scanning:
            self._scan_client.cancel_all_goals()
            self._scanning = False

    def _stop_robot(self):
        self._cmd_vel_pub.publish(Twist())

    # MAIN EXEC LOOP
    def _loop(self, _event):
        # safety interrupts
        if self._state != ESTOP:
            if self._safety_level == CRITICAL:
                self._cancel_nav()
                self._cancel_scan()
                self._stop_robot()
                if self._state not in (AVOID_DYNAMIC, ESTOP):
                    self._state_stack.append(self._state)
                self._set_state(ESTOP)
                return
            if self._safety_level == WARNING and self._state in DELIVERY_STATES:
                self._cancel_nav()
                self._stop_robot()
                self._state_stack.append(self._state)
                self._set_state(AVOID_DYNAMIC)
                return

        if self._state == WAIT_FOR_ORDER:
            return

        if self._state == HOTSPOT_SEARCH_LOOP:
            self._run_search()

        elif self._state == DOCK_AND_DELIVER:
            self._run_dock()

        elif self._state == NAV_TO_HOME:
            self._run_home()

        elif self._state == AVOID_DYNAMIC:
            self._stop_robot()
            if self._safety_level == CLEAR:
                resume = self._state_stack.pop() if self._state_stack else WAIT_FOR_ORDER
                rospy.loginfo('task_coordinator: obstacle cleared — resuming %s', resume)
                self._set_state(resume)

        elif self._state == ESTOP:
            self._stop_robot()
            # ESTOP clears only when a new order is received (handled in _target_qr)

    def _run_search(self):
        if self._qr_match:
            rospy.loginfo('task_coordinator: QR match — transitioning to DOCK_AND_DELIVER')
            self._cancel_nav()
            self._cancel_scan()
            self._set_state(DOCK_AND_DELIVER)
            return

        if not self._tables:
            rospy.logwarn('task_coordinator: no tables configured')
            return

        if not self._mb_available:
            return

        # scanning, wait for result
        if self._scanning:
            if self._scan_client.get_state() in (0, 1):
                return  # still spinning
            result = self._scan_client.get_result()
            self._scanning = False
            if result and result.match_found:
                rospy.loginfo('task_coordinator: scan_table match — DOCK_AND_DELIVER')
                self._set_state(DOCK_AND_DELIVER)
                return
            # no match — nav to next table
            rospy.loginfo('task_coordinator: no QR at this table — moving on')
            table = self._tables[self._table_idx]
            rospy.loginfo('task_coordinator: navigating to %s (%.1f, %.1f)',
                          table['id'], table['x'], table['y'])
            self._send_nav_goal(table['x'], table['y'], table['theta'])
            self._table_idx = (self._table_idx + 1) % len(self._tables)
            return

        mb_state = self._mb.get_state()
        if mb_state in (0, 1):
            return  # navigation still running

        # nav success, scan at this table
        if mb_state == 3 and self._scan_available:
            goal = ScanTableGoal(target_order_id=self._order_id or '')
            self._scan_client.send_goal(goal)
            self._scanning = True
            rospy.loginfo('task_coordinator: arrived at table — scanning for "%s"',
                          self._order_id)
            return

        # no goal yet / nav failed / no scan server: go to next table
        table = self._tables[self._table_idx]
        rospy.loginfo('task_coordinator: navigating to %s (%.1f, %.1f)',
                      table['id'], table['x'], table['y'])
        self._send_nav_goal(table['x'], table['y'], table['theta'])
        self._table_idx = (self._table_idx + 1) % len(self._tables)

    def _run_dock(self):
        if self._dock_start is None:
            self._dock_start = rospy.Time.now()
            rospy.loginfo('task_coordinator: docking — waiting %.1f s', self._dock_wait)
            return

        if (rospy.Time.now() - self._dock_start).to_sec() >= self._dock_wait:
            self._dock_start = None
            self._set_state(NAV_TO_HOME)

    def _run_home(self):
        if not self._mb_available:
            rospy.loginfo('task_coordinator: no move_base — returning to WAIT_FOR_ORDER')
            self._reset_order()
            return

        mb_state = self._mb.get_state()
        if mb_state not in (0, 1):
            # not yet sent or already done
            if self._mb.get_state() == 3:  # SUCCEEDED
                self._reset_order()
                return
            k = self._kitchen
            rospy.loginfo('task_coordinator: returning home (%.1f, %.1f)', k['x'], k['y'])
            self._send_nav_goal(k['x'], k['y'], k['theta'])

    def _reset_order(self):
        rospy.loginfo('task_coordinator: order complete — WAIT_FOR_ORDER')
        self._cancel_scan()
        self._state_stack.clear()
        self._order_id  = None
        self._table_idx = 0
        self._qr_match  = False
        self._set_state(WAIT_FOR_ORDER)


def main():
    TaskCoordinator()
    rospy.spin()


if __name__ == '__main__':
    main()
