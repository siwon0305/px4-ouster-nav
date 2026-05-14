#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class CmdVelOffboard(Node):
    def __init__(self):
        super().__init__('cmd_vel_offboard')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('setpoint_topic', '/mavros/setpoint_velocity/cmd_vel_unstamped')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.4)

        self.declare_parameter('max_xy_speed', 0.6)
        self.declare_parameter('max_z_speed', 0.25)
        self.declare_parameter('max_yaw_rate', 0.4)
        self.declare_parameter('body_frame_cmd', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.setpoint_topic = self.get_parameter('setpoint_topic').value

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.max_xy_speed = float(self.get_parameter('max_xy_speed').value)
        self.max_z_speed = float(self.get_parameter('max_z_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.body_frame_cmd = bool(self.get_parameter('body_frame_cmd').value)

        self.state: Optional[State] = None
        self.pose: Optional[PoseStamped] = None

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.connected_time = None
        self.last_request_time = self.get_clock().now()

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_cb,
            pose_qos
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_cb,
            10
        )

        self.vel_pub = self.create_publisher(
            Twist,
            self.setpoint_topic,
            10
        )

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_cb
        )

        self.get_logger().info('cmd_vel_offboard started.')
        self.get_logger().info(f'Subscribing cmd_vel: {self.cmd_vel_topic}')
        self.get_logger().info(f'Subscribing pose:    {self.pose_topic}')
        self.get_logger().info(f'Publishing setpoint: {self.setpoint_topic}')
        self.get_logger().info(f'body_frame_cmd={self.body_frame_cmd}')

    def state_cb(self, msg: State):
        self.state = msg

    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def yaw_from_pose(self) -> float:
        if self.pose is None:
            return 0.0

        q = self.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        return math.atan2(siny_cosp, cosy_cosp)

    def make_velocity_setpoint(self) -> Twist:
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9

        if age > self.cmd_timeout:
            return Twist()

        cmd = self.last_cmd

        vx_body = self.clamp(
            cmd.linear.x,
            -self.max_xy_speed,
            self.max_xy_speed
        )
        vy_body = self.clamp(
            cmd.linear.y,
            -self.max_xy_speed,
            self.max_xy_speed
        )
        vz = self.clamp(
            cmd.linear.z,
            -self.max_z_speed,
            self.max_z_speed
        )
        yaw_rate = self.clamp(
            cmd.angular.z,
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        out = Twist()

        if self.body_frame_cmd:
            yaw = self.yaw_from_pose()

            out.linear.x = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
            out.linear.y = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body
        else:
            out.linear.x = vx_body
            out.linear.y = vy_body

        out.linear.z = vz
        out.angular.z = yaw_rate

        return out

    def request_offboard(self):
        if not self.mode_client.service_is_ready():
            return

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'OFFBOARD'

        self.mode_client.call_async(req)
        self.get_logger().info('Requested OFFBOARD mode.')

    def request_arm(self):
        if not self.arm_client.service_is_ready():
            return

        req = CommandBool.Request()
        req.value = True

        self.arm_client.call_async(req)
        self.get_logger().info('Requested arm.')

    def timer_cb(self):
        self.vel_pub.publish(self.make_velocity_setpoint())

        if self.state is None:
            return

        if not self.state.connected:
            return

        now = self.get_clock().now()

        if self.connected_time is None:
            self.connected_time = now
            return

        if (now - self.connected_time).nanoseconds * 1e-9 < 2.0:
            return

        if (now - self.last_request_time).nanoseconds * 1e-9 > 2.0:
            if self.state.mode != 'OFFBOARD':
                self.request_offboard()
                self.last_request_time = now
                return

            if not self.state.armed:
                self.request_arm()
                self.last_request_time = now
                return

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOffboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
