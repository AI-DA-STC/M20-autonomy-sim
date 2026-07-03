#!/usr/bin/env python3
"""
Two responsibilities:
1. Bridges 16 per-joint /model/M20/joint/*/cmd_force Float64 topics to a single
   Float64MultiArray for ros2_control's joint_effort_controller.
2. Relays /joint_states → /M20/joint_states (avoids needing topic_tools).
Joint order in JOINT_NAMES must match ros2_control_classic.yaml.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint',
]


class EffortBridge(Node):
    def __init__(self):
        super().__init__('effort_bridge')
        self.forces = [0.0] * len(JOINT_NAMES)

        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/joint_effort_controller/commands', 10)

        self.js_pub = self.create_publisher(JointState, '/M20/joint_states', 10)

        for i, name in enumerate(JOINT_NAMES):
            topic = f'/model/M20/joint/{name}/cmd_force'
            self.create_subscription(
                Float64, topic,
                lambda msg, idx=i: self._force_cb(msg, idx),
                10)

        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.create_timer(0.002, self._publish_efforts)  # 500 Hz

    def _force_cb(self, msg: Float64, idx: int):
        self.forces[idx] = msg.data

    def _js_cb(self, msg: JointState):
        self.js_pub.publish(msg)

    def _publish_efforts(self):
        out = Float64MultiArray()
        out.data = self.forces[:]
        self.effort_pub.publish(out)


def main():
    rclpy.init()
    node = EffortBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
