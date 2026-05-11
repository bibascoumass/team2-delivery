#!/usr/bin/env python

import math
import yaml
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class HotspotVisualizer:
    def __init__(self):
        rospy.init_node("hotspot_visualizer")

        self.hotspots_yaml = rospy.get_param("~hotspots_yaml")
        self.marker_topic = rospy.get_param("~marker_topic", "/hotspot_markers")
        self.publish_rate = rospy.get_param("~publish_rate", 1.0)

        self.frame_id, self.hotspots = self.load_hotspots(self.hotspots_yaml)

        self.pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo("HOTSPOT_VISUALIZER: loaded %d hotspots from %s",
                      len(self.hotspots), self.hotspots_yaml)

    def load_hotspots(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        frame_id = data.get("frame_id", "map")
        hotspots = data.get("hotspots", [])

        if hotspots is None:
            hotspots = []

        return frame_id, hotspots

    def timer_cb(self, _event):
        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for i, hp in enumerate(self.hotspots):
            x = float(hp["x"])
            y = float(hp["y"])
            yaw = float(hp.get("yaw", 0.0))
            name = hp.get("name", "hotspot_%02d" % (i + 1))

            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "hotspot_points"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.08
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.25
            sphere.scale.y = 0.25
            sphere.scale.z = 0.25
            sphere.color.r = 0.1
            sphere.color.g = 0.8
            sphere.color.b = 1.0
            sphere.color.a = 0.95
            arr.markers.append(sphere)

            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "hotspot_headings"
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.05
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color.r = 0.0
            arrow.color.g = 1.0
            arrow.color.b = 0.3
            arrow.color.a = 0.95

            p0 = Point()
            p0.x = x
            p0.y = y
            p0.z = 0.15

            p1 = Point()
            p1.x = x + 0.55 * math.cos(yaw)
            p1.y = y + 0.55 * math.sin(yaw)
            p1.z = 0.15

            arrow.points.append(p0)
            arrow.points.append(p1)
            arr.markers.append(arrow)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = rospy.Time.now()
            text.ns = "hotspot_labels"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.45
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = "%d: %s" % (i + 1, name)
            arr.markers.append(text)

        self.pub.publish(arr)


if __name__ == "__main__":
    try:
        HotspotVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
