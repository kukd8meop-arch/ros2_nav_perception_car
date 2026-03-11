#!/usr/bin/env python3
"""
odom_to_tf_node.py — 从 /odom 话题提取 odom→base_footprint TF 并发布
=====================================================================

Ignition Gazebo 的 odometry-publisher 插件在 Ignition Transport 内部发布
odom→base_footprint 的 TF 变换，但 ros_gz_bridge 无法直接桥接 Ignition
的 TF 到 ROS2 的 /tf 话题（消息类型不兼容）。

本节点的解决方案：
  1. 订阅已通过 ros_gz_bridge 桥接到 ROS2 的 /odom 话题
  2. 从 Odometry 消息中提取位姿（pose）
  3. 发布 odom → base_footprint 的 TF 变换

这样 Nav2 的 AMCL、costmap 等组件就能正常获取 TF 链：
  map → odom → base_footprint → base_link → lidar_frame
        ^^^^ 本节点发布
  ^^^^ AMCL 发布
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )
        self.get_logger().info('odom_to_tf 节点启动：订阅 /odom → 发布 odom→base_footprint TF')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header = msg.header  # frame_id = "odom", stamp = 仿真时间
        t.child_frame_id = msg.child_frame_id  # "base_footprint"

        # 如果 child_frame_id 为空或带 model 前缀，手动修正
        if not t.child_frame_id or '/' in t.child_frame_id:
            t.child_frame_id = 'base_footprint'
        if not t.header.frame_id:
            t.header.frame_id = 'odom'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
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
