#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTf(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('use_message_stamp', False)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.use_message_stamp = bool(self.get_parameter('use_message_stamp').value)

        self.last_pose: Optional[PoseStamped] = None
        self.br = TransformBroadcaster(self)

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

        self.timer = self.create_timer(0.02, self.publish_tf)  # 50 Hz
        
        self.get_logger().info(
            f'Broadcasting TF {self.parent_frame} -> {self.child_frame} '
            f'from {self.pose_topic}'
        )

    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg

    def publish_tf(self):
        if self.last_pose is None:
            return

        msg = self.last_pose

        t = TransformStamped()

        if self.use_message_stamp:
            t.header.stamp = msg.header.stamp
        else:
            # Gazebo /clock 기준으로 TF 시간을 맞추기 위해 node clock을 사용합니다.
            t.header.stamp = self.get_clock().now().to_msg()

        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTf()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()