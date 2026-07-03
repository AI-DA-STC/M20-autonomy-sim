#!/usr/bin/env python3
"""Relay CMU TwistStamped /cmd_vel to M20's plain Twist /cmd_vel_raw."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) < deadband else value


class CmdVelToRawRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_raw_relay')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_raw')
        self.declare_parameter('linear_x_scale', 1.0)
        self.declare_parameter('linear_y_scale', 1.0)
        self.declare_parameter('yaw_scale', 1.0)
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_yaw', 2.0)
        self.declare_parameter('deadband', 0.01)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.linear_x_scale = float(self.get_parameter('linear_x_scale').value)
        self.linear_y_scale = float(self.get_parameter('linear_y_scale').value)
        self.yaw_scale = float(self.get_parameter('yaw_scale').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_yaw = float(self.get_parameter('max_yaw').value)
        self.deadband = float(self.get_parameter('deadband').value)

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.sub = self.create_subscription(TwistStamped, input_topic, self.cmd_cb, 10)

        self.count = 0
        self.get_logger().info(
            f'cmd_vel relay: {input_topic} -> {output_topic}, '
            f'scale=({self.linear_x_scale}, {self.linear_y_scale}, {self.yaw_scale})'
        )

    def cmd_cb(self, msg: TwistStamped):
        out = Twist()
        out.linear.x = clamp(
            apply_deadband(msg.twist.linear.x * self.linear_x_scale, self.deadband),
            self.max_linear,
        )
        out.linear.y = clamp(
            apply_deadband(msg.twist.linear.y * self.linear_y_scale, self.deadband),
            self.max_linear,
        )
        out.angular.z = clamp(
            apply_deadband(msg.twist.angular.z * self.yaw_scale, self.deadband),
            self.max_yaw,
        )

        if not all(math.isfinite(v) for v in (out.linear.x, out.linear.y, out.angular.z)):
            self.get_logger().warn('Dropping non-finite cmd_vel')
            return

        self.pub.publish(out)
        self.count += 1
        if self.count == 1 or self.count % 100 == 0:
            self.get_logger().info(
                f'raw cmd #{self.count}: x={out.linear.x:.3f}, '
                f'y={out.linear.y:.3f}, yaw={out.angular.z:.3f}'
            )


def main():
    rclpy.init()
    node = CmdVelToRawRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
