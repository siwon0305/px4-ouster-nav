#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')

        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        # Cartographer 내부 시간 tick은 100 ns 단위입니다.
        # 1 ns 보정은 중복 시간으로 보일 수 있으므로 1 ms 이상 띄웁니다.
        self.declare_parameter('min_time_step_ns', 1_000_000)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.min_time_step_ns = int(self.get_parameter('min_time_step_ns').value)

        self.last_stamp_ns: Optional[int] = None

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_cb,
            pose_qos
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_pub = TransformBroadcaster(self)

        self.get_logger().info(
            f'Publishing /odom and TF {self.parent_frame} -> {self.child_frame}: '
            f'{self.pose_topic} -> {self.odom_topic}'
        )
        self.get_logger().info(
            f'min_time_step_ns={self.min_time_step_ns}'
        )

    def make_monotonic_stamp(self, msg_stamp=None):
        """
        Cartographer 입력 시간을 Gazebo /clock 기반으로 통일합니다.
        MAVROS msg.header.stamp는 사용하지 않습니다.
        """
        stamp_ns = self.get_clock().now().nanoseconds

        if stamp_ns <= 0:
            stamp_ns = 1 if self.last_stamp_ns is None else self.last_stamp_ns + self.min_time_step_ns

        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + self.min_time_step_ns

        if self.last_stamp_ns is not None and (stamp_ns - self.last_stamp_ns) < self.min_time_step_ns:
            stamp_ns = self.last_stamp_ns + self.min_time_step_ns

        self.last_stamp_ns = stamp_ns
        return Time(nanoseconds=stamp_ns).to_msg()

    def pose_cb(self, msg: PoseStamped):
        stamp = self.make_monotonic_stamp()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id = self.child_frame

        odom.pose.pose = msg.pose

        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame

        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation

        self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()