#!/usr/bin/env python3
"""
FAST-LIO2 /cloud_registered → 地面滤波 → /cloud_registered_filtered

用法（必须用系统 Python 并指定 PYTHONPATH）：
  source /opt/ros/humble/setup.bash
  export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
  /usr/bin/python3 .../ground_filter.py \
    --ros-args -p ground_z_threshold:=-0.4 -p input_cloud:=/cloud_registered -p output_cloud:=/cloud_registered_filtered
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
import numpy as np
import struct


class GroundFilter(Node):
    def __init__(self):
        super().__init__('fastlio_ground_filter')

        self.declare_parameter('input_cloud', '/cloud_registered')
        self.declare_parameter('input_odom', '/Odometry')
        self.declare_parameter('output_cloud', '/cloud_registered_filtered')
        # Z 裁剪阈值（body 帧下，低于此值 → 丢弃。单位 m，默认 -0.4 即
        # Go2 站立时 body 高 ~0.3m，地面在 z_body ≈ -0.35..-0.45）
        self.declare_parameter('ground_z_threshold', -0.35)

        inp = self.get_parameter('input_cloud').value
        odom_t = self.get_parameter('input_odom').value
        out = self.get_parameter('output_cloud').value
        self.thr = self.get_parameter('ground_z_threshold').value

        self.latest_odom = None  # (tx, ty, tz, qx, qy, qz, qw)

        self.sub_cloud = self.create_subscription(
            PointCloud2, inp, self.cloud_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, odom_t, self.odom_cb, 10)
        self.pub = self.create_publisher(PointCloud2, out, 10)
        self.get_logger().info(
            f'Ground filter ready: {inp}→{out}, z_threshold={self.thr:.2f}m (body frame)')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def cloud_cb(self, msg):
        if self.latest_odom is None:
            return  # 尚无里程计，等待

        tx, ty, tz, qx, qy, qz, qw = self.latest_odom

        # 从四元数构建旋转矩阵（w2b = world→body）
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]])
        T = np.array([tx, ty, tz])

        kept = []
        total = 0
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                           skip_nans=True):
            pt_w = np.array([p[0], p[1], p[2]])
            pt_b = R.T @ (pt_w - T)        # world → body
            total += 1
            if pt_b[2] > self.thr:         # 高于阈值 → 保留
                kept.append((p[0], p[1], p[2]))

        out_msg = point_cloud2.create_cloud_xyz32(msg.header, kept)
        self.pub.publish(out_msg)

        if total > 0:
            self.get_logger().info(
                f'filtered {total}→{len(kept)} pts ({100*len(kept)/total:.1f}% kept)',
                throttle_duration_sec=5.0)


def main():
    rclpy.init()
    rclpy.spin(GroundFilter())


if __name__ == '__main__':
    main()
