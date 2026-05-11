#!/usr/bin/env python


import rospy
import cv2
import numpy as np

from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point


class QRDetector:
    def __init__(self):
        rospy.init_node("qr_detector")

        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")

        self.latest_depth = None

        self.qr_pub = rospy.Publisher("/detected_qr", String, queue_size=10)
        self.offset_pub = rospy.Publisher("/qr_offset", Point, queue_size=10)

        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

        rospy.loginfo("qr_detector running")

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn("Depth conversion failed: %s", e)

    def get_depth_at(self, cx, cy):
        if self.latest_depth is None:
            return -1.0

        h, w = self.latest_depth.shape[:2]
        cx = int(np.clip(cx, 0, w - 1))
        cy = int(np.clip(cy, 0, h - 1))

        patch = self.latest_depth[
            max(0, cy - 5):min(h, cy + 6),
            max(0, cx - 5):min(w, cx + 6)
        ]

        patch = patch.astype(np.float32)
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]

        if patch.size == 0:
            return -1.0

        depth = float(np.median(patch))

        if depth > 20.0:
            depth = depth / 1000.0

        return depth

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("Image conversion failed: %s", e)
            return

        h, w = frame.shape[:2]
        objects = decode(frame)

        for obj in objects:
            try:
                data = obj.data.decode("utf-8").strip()
            except Exception:
                continue

            if not data:
                continue

            if obj.polygon and len(obj.polygon) >= 4:
                pts = np.array([(p.x, p.y) for p in obj.polygon], dtype=np.float32)
            else:
                x, y, bw, bh = obj.rect
                pts = np.array(
                    [[x, y], [x + bw, y], [x + bw, y + bh], [x, y + bh]],
                    dtype=np.float32
                )

            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))

            x_error = (cx - w / 2.0) / (w / 2.0)
            area_frac = cv2.contourArea(pts) / float(w * h)
            depth = self.get_depth_at(cx, cy)

            self.qr_pub.publish(String(data=data))

            p = Point()
            p.x = x_error
            p.y = area_frac
            p.z = depth
            self.offset_pub.publish(p)

            rospy.loginfo_throttle(
                0.5,
                "Detected QR: %s | x_error=%.3f area=%.5f depth=%.3f",
                data,
                x_error,
                area_frac,
                depth,
            )


if __name__ == "__main__":
    QRDetector()
    rospy.spin()
