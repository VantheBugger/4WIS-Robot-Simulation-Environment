#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm

import ros_numpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class PointCloudCarFilterTF:
    def __init__(self):
        rospy.init_node("pointcloud_car_filter_tf")

        # =============================
        # 参数
        # =============================
        self.target_frame = rospy.get_param("~target_frame", "base_link")

        # 车体尺寸（base_link 下）
        self.car_x_min = rospy.get_param("~car_x_min", -0.8)
        self.car_x_max = rospy.get_param("~car_x_max",  0.8)
        self.car_y_min = rospy.get_param("~car_y_min", -0.5)
        self.car_y_max = rospy.get_param("~car_y_max",  0.5)
        self.car_z_min = rospy.get_param("~car_z_min", -0.3)
        self.car_z_max = rospy.get_param("~car_z_max",  1.0)

        # =============================
        # TF
        # =============================
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(0.01))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # =============================
        # ROS I/O
        # =============================
        self.sub = rospy.Subscriber(
            "velodyne_points", PointCloud2,
            self.callback, queue_size=1
        )

        self.pub = rospy.Publisher(
            "points_filtered", PointCloud2,
            queue_size=1
        )

        rospy.loginfo("[PointCloudCarFilterTF] Started (using ros_numpy).")

    def callback(self, msg: PointCloud2):

        # --------------------------------
        # 1. TF：点云 → base_link
        # --------------------------------
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.01)
            )
            cloud_bl = tf2_sm.do_transform_cloud(msg, transform)
        except Exception as e:
            rospy.logwarn_throttle(1.0, "TF failed: %s", str(e))
            return

        # --------------------------------
        # 2. PointCloud2 → numpy（高效转换）
        # --------------------------------
        try:
            # 使用 ros_numpy 快速转换（比 pc2.read_points 快 10-100 倍）
            pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(cloud_bl)
            
            # 提取 xyz 坐标
            points = np.zeros((pc_array.shape[0], 3), dtype=np.float32)
            points[:, 0] = pc_array['x']
            points[:, 1] = pc_array['y']
            points[:, 2] = pc_array['z']
            
            # 移除 NaN 点
            valid_mask = ~np.isnan(points).any(axis=1)
            points = points[valid_mask]
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, "Point cloud conversion failed: %s", str(e))
            return

        if points.shape[0] == 0:
            return

        # --------------------------------
        # 3. 去除车体矩形（向量化操作）
        # --------------------------------
        mask = ~(
            (points[:, 0] > self.car_x_min) & (points[:, 0] < self.car_x_max) &
            (points[:, 1] > self.car_y_min) & (points[:, 1] < self.car_y_max) &
            (points[:, 2] > self.car_z_min) & (points[:, 2] < self.car_z_max)
        )
        points = points[mask]
        
        if points.shape[0] == 0:
            return

        # --------------------------------
        # 4. numpy → PointCloud2（高效转换）
        # --------------------------------
        # 创建结构化数组
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        structured_array = np.zeros(points.shape[0], dtype=dtype)
        structured_array['x'] = points[:, 0]
        structured_array['y'] = points[:, 1]
        structured_array['z'] = points[:, 2]
        
        # 转换为 PointCloud2
        cloud_out = ros_numpy.point_cloud2.array_to_pointcloud2(
            structured_array,
            stamp=msg.header.stamp,
            frame_id=self.target_frame
        )

        # 计算处理时间
        self.pub.publish(cloud_out)


if __name__ == "__main__":
    PointCloudCarFilterTF()
    rospy.spin()