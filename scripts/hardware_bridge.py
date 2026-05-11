#!/usr/bin/env python
import math

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import serial
except ImportError:
    serial = None

SIM_MODE_PARAM = '~sim_mode'  # false = real hardware stop path, true = simulated odom path
WATCHDOG_TIMEOUT = 0.5  # seconds - ARBITRARY
DEFAULT_CMD_FORMAT = 'CMD {linear_x:.3f} {angular_z:.3f}\n'
DEFAULT_STOP_COMMAND = 'CMD 0.000 0.000\n'


def cb_cmd_vel(msg, state):
    state['last_cmd_time'] = rospy.Time.now()
    state['cmd'] = msg
    rospy.logdebug('hardware_bridge: /cmd_vel v=%.3f omega=%.3f', msg.linear.x, msg.angular.z)


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def safe_cmd(state, now):
    if (now - state['last_cmd_time']).to_sec() > WATCHDOG_TIMEOUT:
        return Twist()
    return state['cmd']


def open_serial_port():
    if serial is None:
        raise RuntimeError('python-serial is required for real hardware mode')

    port = rospy.get_param('~serial_port', '/dev/ttyACM0')
    baud = rospy.get_param('~baud_rate', 115200)
    timeout = rospy.get_param('~serial_timeout', 0.02)
    rospy.loginfo('hardware_bridge: opening serial port %s @ %s', port, baud)
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def format_cmd(cmd):
    fmt = rospy.get_param('~cmd_format', DEFAULT_CMD_FORMAT)
    return fmt.format(
        linear_x=cmd.linear.x,
        linear_y=cmd.linear.y,
        angular_z=cmd.angular.z)


def write_serial_line(serial_port, text):
    if not text.endswith('\n'):
        text += '\n'
    serial_port.write(text.encode('ascii'))


def send_real_cmd(serial_port, cmd, state):
    text = format_cmd(cmd)
    if text != state.get('last_serial_cmd'):
        write_serial_line(serial_port, text)
        state['last_serial_cmd'] = text


def send_real_stop(serial_port, state):
    text = rospy.get_param('~stop_command', DEFAULT_STOP_COMMAND)
    if text != state.get('last_serial_cmd'):
        write_serial_line(serial_port, text)
        state['last_serial_cmd'] = text


def parse_odom_line(line):
    """Accepts: ODOM x y yaw vx wz  or  x y yaw vx wz."""
    parts = line.strip().replace(',', ' ').split()
    if not parts:
        return None
    if parts[0].upper() == 'ODOM':
        parts = parts[1:]
    if len(parts) < 5:
        return None
    try:
        x, y, yaw, vx, wz = [float(v) for v in parts[:5]]
    except ValueError:
        return None
    return x, y, yaw, vx, wz


def publish_odom(odom_pub, tf_broadcaster, now, x, y, yaw, vx, wz):
    qx, qy, qz, qw = yaw_to_quaternion(yaw)

    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = qx
    odom.pose.pose.orientation.y = qy
    odom.pose.pose.orientation.z = qz
    odom.pose.pose.orientation.w = qw
    odom.twist.twist.linear.x = vx
    odom.twist.twist.angular.z = wz

    odom_pub.publish(odom)
    tf_broadcaster.sendTransform(
        (x, y, 0.0),
        (qx, qy, qz, qw),
        now,
        'base_link',
        'odom')


def read_real_odom(serial_port, odom_pub, tf_broadcaster):
    raw = serial_port.readline()
    if not raw:
        return
    try:
        line = raw.decode('ascii', errors='replace').strip()
    except AttributeError:
        line = str(raw).strip()
    parsed = parse_odom_line(line)
    if parsed is None:
        rospy.logdebug('hardware_bridge: ignoring serial line: %s', line)
        return
    publish_odom(odom_pub, tf_broadcaster, rospy.Time.now(), *parsed)


def publish_sim_odom(state, odom_pub, tf_broadcaster, now, cmd):
    if now <= state['last_odom_time']:
        return

    dt = (now - state['last_odom_time']).to_sec()
    state['last_odom_time'] = now

    linear_x = cmd.linear.x
    angular_z = cmd.angular.z

    yaw_mid = state['yaw'] + 0.5 * angular_z * dt
    state['x'] += linear_x * math.cos(yaw_mid) * dt
    state['y'] += linear_x * math.sin(yaw_mid) * dt
    state['yaw'] += angular_z * dt

    publish_odom(odom_pub, tf_broadcaster, now,
                 state['x'], state['y'], state['yaw'],
                 linear_x, angular_z)


def main():
    rospy.init_node('hardware_bridge')

    sim_mode = rospy.get_param(SIM_MODE_PARAM, False)
    rospy.loginfo('hardware_bridge: sim_mode=%s', sim_mode)
    serial_port = None
    if not sim_mode:
        serial_port = open_serial_port()

    now = rospy.Time.now()
    state = {
        'last_cmd_time': now,
        'last_odom_time': now,
        'cmd': Twist(),
        'x': rospy.get_param('~initial_x', 0.0),
        'y': rospy.get_param('~initial_y', 0.0),
        'yaw': rospy.get_param('~initial_yaw', 0.0),
        'last_serial_cmd': None,
    }

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    robot_cmd_pub = rospy.Publisher('/robot_cmd_vel', Twist, queue_size=10)
    tf_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber('/cmd_vel', Twist, cb_cmd_vel, callback_args=state)

    rate = rospy.Rate(20)  # TODO - ARBITRARY
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        elapsed = (now - state['last_cmd_time']).to_sec()
        cmd = safe_cmd(state, now)
        if elapsed > WATCHDOG_TIMEOUT:
            if sim_mode:
                rospy.logwarn_throttle(5.0, 'hardware_bridge: watchdog timeout - simulated motors stopped')
            else:
                rospy.logwarn_throttle(5.0, 'hardware_bridge: watchdog timeout - cutting motor power')
                send_real_stop(serial_port, state)

        if sim_mode:
            robot_cmd_pub.publish(cmd)
            publish_sim_odom(state, odom_pub, tf_broadcaster, now, cmd)
        else:
            send_real_cmd(serial_port, cmd, state)
            read_real_odom(serial_port, odom_pub, tf_broadcaster)

        rate.sleep()


if __name__ == '__main__':
    main()
