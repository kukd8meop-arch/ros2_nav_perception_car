#!/usr/bin/env python3
"""
scan_frame_fix_node.py — 修正 Ignition Gazebo 的 LaserScan frame_id
====================================================================

Ignition Gazebo 使用 scoped frame 命名，例如：
  rosorin/base_footprint/lidar

但 Nav2 / AMCL 的 costmap 和 TF 树期望的是 URDF 中定义的 link 名：
  lidar_frame

本节点订阅原始 scan 话题，修正 frame_id 后重新发布。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanFrameFixNode(Node):
    def __init__(self):
        super().__init__('scan_frame_fix')

        # 参数：目标 frame_id
        self.declare_parameter('target_frame', 'lidar_frame')
        self.target_frame = self.get_parameter('target_frame').value

        # 订阅 bridge 输出的原始 scan（frame_id 为 scoped name）
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.sub = self.create_subscription(
            LaserScan,
            '/scan_raw_gz',   # bridge 输出（重命名后）
            self.scan_cb,
            sensor_qos,
        )

        # 发布修正后的 scan
        self.pub = self.create_publisher(
            LaserScan,
            '/scan_raw',      # Nav2 / costmap 使用的话题
            sensor_qos,
        )

        self.get_logger().info(
            f'scan_frame_fix: /scan_raw_gz → /scan_raw  '
            f'(frame_id → {self.target_frame})'
        )

    def scan_cb(self, msg: LaserScan):
        msg.header.frame_id = self.target_frame
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
