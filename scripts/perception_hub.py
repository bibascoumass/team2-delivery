#!/usr/bin/env python3
import math
import rospy
import actionlib
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from team2_delivery.msg import QRInfo
from team2_delivery.msg import ScanTableAction, ScanTableResult, ScanTableFeedback
from pyzbar.pyzbar import decode as pyzbar_decode
import numpy as np
import cv2
"""
handles QR detection
uses ScanTable action server to spin in place while continuously reading from camera
publishes /qr_data (QRInfo.msg) on every processed RGB frame.
"""


# Decode sensor_msgs/Image to a BGR8 numpy array. CvBridge.imgmsg_to_cv2 broken due to issue with cv_bridge_boost.so
def imgmsg_to_bgr8(msg): 
    enc = msg.encoding
    if enc in ('bgr8', 'rgb8'):
        channels = 3
    elif enc in ('bgra8', 'rgba8'):
        channels = 4
    elif enc == 'mono8':
        channels = 1
    else:
        raise ValueError('imgmsg_to_bgr8: unsupported encoding ' + enc)

    arr = np.frombuffer(msg.data, dtype=np.uint8)
    row_bytes = msg.width * channels
    if msg.step != row_bytes:
        arr = arr.reshape(msg.height, msg.step)[:, :row_bytes]
    if channels == 1:
        arr = arr.reshape(msg.height, msg.width)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    arr = arr.reshape(msg.height, msg.width, channels)
    if enc == 'bgr8':
        return arr.copy()
    if enc == 'rgb8':
        return arr[:, :, ::-1].copy()
    if enc == 'bgra8':
        return arr[:, :, :3].copy()
    # rgba8
    return arr[:, :, [2, 1, 0]].copy()

class PerceptionHub:
    def __init__(self):
        rospy.init_node('perception_hub')

        settings = rospy.get_param('/delivery_settings', {})
        self._scan_max_deg = settings.get('scan_max_angle_deg', 360.0)
        self._scan_speed_degs = settings.get('scan_angular_vel_degs', 15.0)

        self._current_order = None   # set by /target_qr
        self._qr_match = False  # set by image callback
        self._qr_detected = False

        self._qr_pub = rospy.Publisher('/qr_data', QRInfo, queue_size=10)
        self._depth_cloud_pub = rospy.Publisher('/depth_cloud', PointCloud2, queue_size=10)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/target_qr', String, self._target_qr)
        rospy.Subscriber('/camera/color/image_raw', Image, self._rgb_image)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self._depth_cloud)

        self._server = actionlib.SimpleActionServer(
            'scan_table',
            ScanTableAction,
            execute_cb=self._handle_scan_table,
            auto_start=False,
        )
        self._server.start()

        rospy.loginfo('perception_hub: ready')

    def _target_qr(self, msg):
        self._current_order = msg.data.strip() or None
        self._qr_match = False
        self._qr_detected = False

    def _rgb_image(self, msg):
        qr_info = QRInfo()
        qr_info.order_id = self._current_order or ''
        qr_info.is_detected = False
        qr_info.is_match = False

        try:
            frame = imgmsg_to_bgr8(msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, 'perception_hub: image conversion failed: %s', e)
            self._qr_pub.publish(qr_info)
            return

        objects = pyzbar_decode(frame)
        for obj in objects:
            try:
                data = obj.data.decode('utf-8').strip()
            except Exception:
                continue
            if not data:
                continue

            qr_info.is_detected = True
            if self._current_order and data.lower() == self._current_order.lower():
                qr_info.is_match = True

            rospy.loginfo_throttle(1.0, 'perception_hub: QR detected="%s" match=%s',
                                   data, qr_info.is_match)
            break  # report first decoded QR per frame

        self._qr_match = qr_info.is_match
        self._qr_detected = qr_info.is_detected
        self._qr_pub.publish(qr_info)

    def _depth_cloud(self, msg):
        # pass through for nav_manager depth fusion
        self._depth_cloud_pub.publish(msg)

    # SCAN TABLE ACTION
    def _handle_scan_table(self, goal):
        target = goal.target_order_id.strip()
        rospy.loginfo('perception_hub: scan_table — target="%s"', target)

        angular_vel = math.radians(self._scan_speed_degs)  # rad/s
        total_rad = math.radians(self._scan_max_deg)
        rotated = 0.0
        dt = 0.1  # 10 Hz spin loop
        rate = rospy.Rate(1.0 / dt)

        twist = Twist()
        twist.angular.z = angular_vel
        feedback = ScanTableFeedback()

        match_found = False
        while rotated < total_rad and not rospy.is_shutdown():
            if self._server.is_preempt_requested():
                self._server.set_preempted()
                self._cmd_vel_pub.publish(Twist())
                return

            self._cmd_vel_pub.publish(twist)
            rotated += angular_vel * dt

            feedback.search_progress = math.degrees(rotated)
            self._server.publish_feedback(feedback)

            if self._qr_match:
                match_found = True
                rospy.loginfo('perception_hub: QR match during scan at %.1f deg',
                              math.degrees(rotated))
                break

            rate.sleep()

        self._cmd_vel_pub.publish(Twist())  # stop

        result = ScanTableResult()
        result.match_found = match_found
        self._server.set_succeeded(result)
        rospy.loginfo('perception_hub: scan_table done — match_found=%s', match_found)


def main():
    PerceptionHub()
    rospy.spin()


if __name__ == '__main__':
    main()
