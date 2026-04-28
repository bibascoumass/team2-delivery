#!/usr/bin/env python3
import rospy
import actionlib
from sensor_msgs.msg import Image, PointCloud2
from team2_delivery.msg import QRInfo
from team2_delivery.msg import ScanTableAction, ScanTableResult, ScanTableFeedback
# handles QR detection, pose estimation, and 3D obstacle filtering 

def cb_rgb_image(msg):
    rospy.logdebug('perception_hub: /camera/color/image_raw received')


def cb_depth_cloud(msg):
    rospy.logdebug('perception_hub: /camera/depth/color/points received')


def handle_scan_table(goal, server, qr_pub):
    # ACTION SERVER CALLBACK 
    rospy.loginfo('perception_hub: goal received — target_order_id=%s', goal.target_order_id)
    feedback = ScanTableFeedback()
    result   = ScanTableResult()

    feedback.search_progress = 0.0
    server.publish_feedback(feedback)

    result.match_found = False
    server.set_succeeded(result)
    rospy.loginfo(f'perception_hub: ScanTable complete — match_found={match_found}')


def main():
    rospy.init_node('perception_hub')

    qr_pub = rospy.Publisher('/qr_data', QRInfo, queue_size=10)
    depth_cloud_pub = rospy.Publisher('/depth_cloud', PointCloud2, queue_size=10)

    rospy.Subscriber('/camera/color/image_raw', Image, cb_rgb_image)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, cb_depth_cloud)

    # ScanTable server
    scan_server = actionlib.SimpleActionServer(
        'scan_table',
        ScanTableAction,
        execute_cb=lambda goal: handle_scan_table(goal, scan_server, qr_pub),
        auto_start=False,
    )
    scan_server.start()

    rospy.loginfo('perception_hub init')
    # TODO - QR detection logic
    rospy.spin()


if __name__ == '__main__':
    main()
