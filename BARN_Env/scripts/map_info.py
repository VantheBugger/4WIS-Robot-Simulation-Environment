#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os

from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class DifficultyRVizNode(object):
    def __init__(self, metrics, path):

        # ========= Publisher =========
        self.text_pub = rospy.Publisher(
            "/difficulty_text",
            Marker,
            queue_size=1,
            latch=True
        )

        self.path_pub = rospy.Publisher(
            "/difficulty_path",
            Path,
            queue_size=1,
            latch=True
        )

        # ========= Marker =========
        self.metrics = metrics
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "difficulty"
        self.marker.id = 0
        self.marker.type = Marker.TEXT_VIEW_FACING
        self.marker.action = Marker.ADD

        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 2.0

        self.marker.scale.z = 0.4
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0

        self.marker.lifetime = rospy.Duration(0)

        # ========= Path =========
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        for p in path:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(p[0])*0.15-4.5
            ps.pose.position.y = float(p[1])*0.15+5.4
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            self.path_msg.poses.append(ps)

        # ========= Timer =========
        self.timer = rospy.Timer(
            rospy.Duration(1),
            self.publish_all
        )

        rospy.loginfo("Difficulty RViz node started.")

    def publish_all(self, event):
        now = rospy.Time.now()

        # ---- Text ----
        self.marker.header.stamp = now
        self.marker.text = (
            "Map Difficulty Metrics\n"
            "----------------------\n"
            "Dis to Obs    : {:.2f}\n"
            "Visibility    : {:.2f}\n"
            "Dispersion    : {:.2f}\n"
            "Dimension     : {:.2f}\n"
            "Tortuosity    : {:.2f}"
        ).format(
            self.metrics["dco"],
            self.metrics["visibility"],
            self.metrics["dispersion"],
            self.metrics["dimension"],
            self.metrics["tortuosity"]
        )

        self.text_pub.publish(self.marker)

        # ---- Path ----
        self.path_msg.header.stamp = now
        self.path_pub.publish(self.path_msg)


if __name__ == "__main__":
    rospy.init_node("difficulty_rviz")

    # ========= Load metrics =========
    metrics_path = rospy.get_param("~metrics_npy", "")
    if not os.path.exists(metrics_path):
        rospy.logerr("Metrics npy not found: %s", metrics_path)
        rospy.signal_shutdown("Missing metrics npy")

    metrics_data = np.load(metrics_path, allow_pickle=True)
    if isinstance(metrics_data, np.ndarray) and metrics_data.dtype == object:
        metrics = metrics_data.item()
    else:
        metrics = {
            "dco": metrics_data[0],
            "visibility": metrics_data[1],
            "dispersion": metrics_data[2],
            "dimension": metrics_data[3],
            "tortuosity": metrics_data[4],
        }

    # ========= Load path =========
    path_path = rospy.get_param("~path_npy", "")
    if not os.path.exists(path_path):
        rospy.logerr("Path npy not found: %s", path_path)
        rospy.signal_shutdown("Missing path npy")

    path = np.load(path_path)
    path = np.asarray(path, dtype=float)
    DifficultyRVizNode(metrics, path)
    rospy.spin()
