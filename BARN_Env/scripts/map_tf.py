#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg


def main():
    rospy.init_node("world_to_map_static_tf")

    world_frame = rospy.get_param("~world_frame", "world")
    map_frame   = rospy.get_param("~map_frame", "map")

    offset_y = rospy.get_param("~map_offset_y", -0.15)

    br = tf2_ros.StaticTransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = world_frame
    t.child_frame_id = map_frame

    t.transform.translation.x = 0.0
    t.transform.translation.y = offset_y
    t.transform.translation.z = 0.0

    # 无旋转
    t.transform.rotation.w = 1.0

    br.sendTransform(t)

    rospy.loginfo("===================================")
    rospy.loginfo("Static TF published")
    rospy.loginfo(" %s -> %s  (y = %.3f m)", world_frame, map_frame, offset_y)
    rospy.loginfo("===================================")

    rospy.spin()


if __name__ == "__main__":
    main()
