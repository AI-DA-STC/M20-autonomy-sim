#!/usr/bin/python3
"""
Gazebo Controller for M20 Robot - Multi-Robot Support

This node bridges between Gazebo simulation and the rl_deploy control system.
It converts standard ROS2 messages from Gazebo to the DDS format expected by rl_deploy.

For multi-robot support:
- robot_name: The model name in Gazebo (e.g., "M20_A", "M20_B")
- namespace: ROS2 namespace for DDS topics (set via launch file)

Topics subscribed (Gazebo-specific, use robot_name):
- /world/{world_name}/model/{robot_name}/joint_state -> remapped to /{robot_name}/joint_states
- /model/{robot_name}/link/base_link/sensor/imu_sensor/imu -> remapped to /{robot_name}/IMU

Topics published (DDS format, use ROS2 namespace):
- /JOINTS_DATA (namespaced to /{namespace}/JOINTS_DATA)
- /IMU_DATA (namespaced to /{namespace}/IMU_DATA)
"""

import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState, Imu, PointCloud2
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64, Float32MultiArray

# Import custom messages
from drdds.msg import (
    ImuData, JointsData, JointsDataCmd, MetaType,
    ImuDataValue, JointsDataValue, JointData, JointDataCmd
)


# Calibration parameters — must match M20Interface init values exactly so that
# the two-stage transform cancels: joint_pos_rl = q_gazebo (identity mapping).
#
# DdsInterface reads:  joint_pos_rl = dds_pos * joint_dir_m20 + offset_rad_m20
# gazebo publishes:    dds_pos      = (q_gazebo - POS_OFFSET_RAD) * JOINT_DIR
#
# For identity: JOINT_DIR == joint_dir_m20  AND  POS_OFFSET_DEG == init_pos_offset_m20
# M20Interface init_pos_offset = {-25,-131,160,0, 25,-131,160,0, -25,131,-160,0, 25,131,-160,0}
# M20Interface joint_dir       = { 1,  1, -1, 1,  1,  -1,  1,-1,  -1,  1,  -1, 1,  -1, -1,  1,-1}
JOINT_DIR = np.array([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1], dtype=np.float32)
POS_OFFSET_DEG = np.array([-25, -131, 160, 0,
                             25, -131, 160, 0,
                            -25,  131,-160, 0,
                             25,  131,-160, 0], dtype=np.float32)
POS_OFFSET_RAD = POS_OFFSET_DEG / 180.0 * np.pi

# Initial joint angles (rl/Gazebo space, same due to identity mapping).
# Matches Mujoco JOINT_INIT: crouched pose with hipy≈-1.16 rad, knee≈2.76 rad for FL/FR,
# and the opposite signs for HL/HR (mirrored kinematics).
# gazebo_controller pre-holds this pose until rl_deploy sends its first command.
JOINT_INIT_GAZEBO = np.array([
    -0.0, -1.16,  2.76, 0.0,   # FL: hipx, hipy, knee, wheel
     0.0, -1.16,  2.76, 0.0,   # FR
    -0.0,  1.16, -2.76, 0.0,   # HL
     0.0,  1.16, -2.76, 0.0,   # HR
], dtype=np.float32)

# PD gains for holding JOINT_INIT before rl_deploy takes control
HOLD_KP = np.array([200., 200., 200., 0.,
                    200., 200., 200., 0.,
                    200., 200., 200., 0.,
                    200., 200., 200., 0.], dtype=np.float32)
HOLD_KD = np.array([4., 4., 4., 1.,
                    4., 4., 4., 1.,
                    4., 4., 4., 1.,
                    4., 4., 4., 1.], dtype=np.float32)

# Per-joint torque limits [Nm]: hipx=32.4, hipy=76.4, knee=76.4, wheel=30.0 (×4 legs)
TORQUE_LIMITS = np.array([
    32.4, 76.4, 76.4, 30.0,   # FL
    32.4, 76.4, 76.4, 30.0,   # FR
    32.4, 76.4, 76.4, 30.0,   # HL
    32.4, 76.4, 76.4, 30.0,   # HR
], dtype=np.float32)

# Joint names in order (16 DOF)
JOINT_NAMES = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
]


class GazeboControllerNode(Node):
    def __init__(self):
        super().__init__('gazebo_controller')

        # Declare parameters
        self.declare_parameter(
            'robot_name',
            'M20',
            ParameterDescriptor(description='Robot model name in Gazebo simulation')
        )
        self.declare_parameter(
            'world_name',
            'Edifice',
            ParameterDescriptor(description='World name in Gazebo simulation')
        )
        self.declare_parameter(
            'wheel_kd_scale',
            6.0,
            ParameterDescriptor(description='Simulation-only multiplier for wheel velocity damping/drive torque')
        )
        self.declare_parameter(
            'wheel_vel_scale',
            1.0,
            ParameterDescriptor(description='Simulation-only multiplier for wheel velocity commands')
        )

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.wheel_kd_scale = self.get_parameter('wheel_kd_scale').get_parameter_value().double_value
        self.wheel_vel_scale = self.get_parameter('wheel_vel_scale').get_parameter_value().double_value

        self.dof_num = 16

        # Control command buffers — all zero until rl_deploy sends the first command.
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)

        # Current state buffers — pre-populated with JOINT_INIT so rl_deploy
        # sees the correct crouched pose immediately on startup.
        self.joint_positions = JOINT_INIT_GAZEBO.copy()
        self.joint_velocities = np.zeros(self.dof_num, dtype=np.float32)
        self.joint_efforts = np.zeros(self.dof_num, dtype=np.float32)
        self.effective_vel_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.effective_kd_cmd = np.zeros((self.dof_num, 1), np.float32)

        # True once rl_deploy sends its first JointsDataCmd.
        # Until then we apply a PD hold at Gazebo's spawned pose so the robot
        # does not move just because the bridge/controller started.
        self.cmd_received = False
        self.hold_position = None

        # IMU state
        self.imu_orientation = Quaternion()
        self.imu_angular_velocity = np.zeros(3, dtype=np.float32)
        self.imu_linear_acceleration = np.zeros(3, dtype=np.float32)

        self.timestamp = 0.0
        self.joints_received = False
        self.imu_received = False

        self.get_logger().info(f"[INFO] Gazebo Controller initialized for robot '{self.robot_name}', dof = {self.dof_num}")
        self.get_logger().info(f"[INFO] Wheel kd scale = {self.wheel_kd_scale:.2f}")
        self.get_logger().info(f"[INFO] Wheel velocity scale = {self.wheel_vel_scale:.2f}")

        # Publishers for custom messages (DDS format)
        # These will be namespaced by ROS2 launch namespace
        self.imu_pub = self.create_publisher(ImuData, 'IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, 'JOINTS_DATA', 200)

        self.get_logger().info(f"[INFO] Publishing to IMU_DATA and JOINTS_DATA (will be namespaced)")
        self.exec_debug_pub = self.create_publisher(Float32MultiArray, f'/{self.robot_name}/exec_debug', 10)

        # Subscriber for custom joint commands (DDS format)
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            'JOINTS_CMD',
            self._cmd_callback,
            50
        )

        # Gazebo bridge subscribers (receive state from Gazebo)
        # These use the robot_name for Gazebo-specific topics
        joint_states_topic = f'/{self.robot_name}/joint_states'
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_states_topic,
            self._joint_state_callback,
            10
        )
        self.get_logger().info(f"[INFO] Subscribed to {joint_states_topic}")

        imu_topic = f'/{self.robot_name}/IMU'
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self._imu_callback,
            10
        )
        self.get_logger().info(f"[INFO] Subscribed to {imu_topic}")

        front_lidar_topic = f'/{self.robot_name}/LIDAR/FRONT'
        self.front_lidar_sub = self.create_subscription(
            PointCloud2,
            front_lidar_topic,
            self._front_lidar_callback,
            10
        )
        self.get_logger().info(f"[INFO] Subscribed to {front_lidar_topic}")

        rear_lidar_topic = f'/{self.robot_name}/LIDAR/REAR'
        self.rear_lidar_sub = self.create_subscription(
            PointCloud2,
            rear_lidar_topic,
            self._rear_lidar_callback,
            10
        )
        self.get_logger().info(f"[INFO] Subscribed to {rear_lidar_topic}")

        # LiDAR data buffers
        self.front_lidar_data = None
        self.rear_lidar_data = None
        self.front_lidar_received = False
        self.rear_lidar_received = False

        # Gazebo bridge publishers (send force commands to each joint)
        # These use the robot_name for Gazebo-specific topics
        self.joint_force_pubs = []
        for joint_name in JOINT_NAMES:
            topic = f'/model/{self.robot_name}/joint/{joint_name}/cmd_force'
            pub = self.create_publisher(Float64, topic, 10)
            self.joint_force_pubs.append(pub)
        self.get_logger().info(f"[INFO] Publishing force commands to /model/{self.robot_name}/joint/*/cmd_force")

        # Timer to publish custom messages at 200 Hz
        self.publish_timer = self.create_timer(0.005, self._publish_robot_state)

        # Timer to send commands to Gazebo at 1000 Hz
        self.control_timer = self.create_timer(0.001, self._send_joint_commands)
        self.debug_timer = self.create_timer(0.1, self._publish_exec_debug)

        self.get_logger().info(f"[INFO] Gazebo Controller ready for robot '{self.robot_name}'")

    def _publish_exec_debug(self):
        """Debug wheel command/response close to the Gazebo actuator layer.

        data layout:
        [cmd_received,
         effective_wheel_vel_cmd(FL,FR,HL,HR),
         wheel_vel_actual(FL,FR,HL,HR),
         wheel_torque(FL,FR,HL,HR),
         wheel_kd(FL,FR,HL,HR)]
        """
        wheel_idx = [3, 7, 11, 15]
        msg = Float32MultiArray()
        msg.data = [1.0 if self.cmd_received else 0.0]
        msg.data.extend(float(self.effective_vel_cmd[i, 0]) for i in wheel_idx)
        msg.data.extend(float(self.joint_velocities[i]) for i in wheel_idx)
        msg.data.extend(float(self.joint_efforts[i]) for i in wheel_idx)
        msg.data.extend(float(self.effective_kd_cmd[i, 0]) for i in wheel_idx)
        self.exec_debug_pub.publish(msg)

    def _cmd_callback(self, msg: JointsDataCmd):
        """Convert received (published) positions/velocities to internal (raw)"""
        if len(msg.data.joints_data) != 16:
            self.get_logger().warn("Received JointsDataCmd with incorrect number of joints")
            return

        pub_pos = np.zeros(self.dof_num, dtype=np.float32)
        pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp if not np.isnan(joint_cmd.kp) else 0.0
            self.kd_cmd[i] = joint_cmd.kd if not np.isnan(joint_cmd.kd) else 0.0
            pub_pos[i] = joint_cmd.position if not np.isnan(joint_cmd.position) else 0.0
            pub_vel[i] = joint_cmd.velocity if not np.isnan(joint_cmd.velocity) else 0.0
            self.tau_ff[i] = joint_cmd.torque if not np.isnan(joint_cmd.torque) else 0.0

        # Convert: raw = published * dir + offset_rad
        self.pos_cmd.flat = pub_pos * JOINT_DIR + POS_OFFSET_RAD
        self.vel_cmd.flat = pub_vel * JOINT_DIR

    def _joint_state_callback(self, msg: JointState):
        """Receive joint states from Gazebo"""
        first_joint_state = not self.joints_received
        if first_joint_state:
            self.get_logger().info(f"[INFO] Received first joint state message with {len(msg.name)} joints")
            self.joints_received = True

        # Map joint states to our order
        for i, name in enumerate(JOINT_NAMES):
            try:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.joint_velocities[i] = msg.velocity[idx]
                if len(msg.effort) > idx:
                    self.joint_efforts[i] = msg.effort[idx]
            except (ValueError, IndexError):
                pass

        if first_joint_state:
            self.hold_position = self.joint_positions.copy()

    def _imu_callback(self, msg: Imu):
        """Receive IMU data from Gazebo"""
        if not self.imu_received:
            self.get_logger().info("[INFO] Received first IMU message")
            self.imu_received = True

        self.imu_orientation = msg.orientation
        self.imu_angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)
        self.imu_linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float32)

    def _front_lidar_callback(self, msg: PointCloud2):
        """Receive front LiDAR point cloud data from Gazebo"""
        if not self.front_lidar_received:
            self.get_logger().info(f"[INFO] Received first front LiDAR message: {msg.width}x{msg.height} points")
            self.front_lidar_received = True
        self.front_lidar_data = msg

    def _rear_lidar_callback(self, msg: PointCloud2):
        """Receive rear LiDAR point cloud data from Gazebo"""
        if not self.rear_lidar_received:
            self.get_logger().info(f"[INFO] Received first rear LiDAR message: {msg.width}x{msg.height} points")
            self.rear_lidar_received = True
        self.rear_lidar_data = msg

    def _send_joint_commands(self):
        """Calculate and send joint torque commands to Gazebo"""
        # Current joint state
        q = self.joint_positions.reshape(-1, 1)
        dq = self.joint_velocities.reshape(-1, 1)

        if not self.cmd_received:
            # Hold JOINT_INIT until rl_deploy sends a command with kp > 0.
            # idle_state has kp=0, so the hold stays active through idle.
            # StandUp/RLControl have kp=200, which releases the hold.
            if np.any(self.kp_cmd > 0):
                self.get_logger().info("[INFO] kp > 0 detected — handing control to rl_deploy")
                self.cmd_received = True
            else:
                if self.hold_position is None:
                    self.hold_position = self.joint_positions.copy()
                target = self.hold_position.reshape(-1, 1)
                kp = HOLD_KP.reshape(-1, 1)
                kd = HOLD_KD.reshape(-1, 1)
                input_tq = kp * (target - q) + kd * (0.0 - dq)
                # Clamp and publish, then return early
                input_tq = np.clip(input_tq.flatten(), -TORQUE_LIMITS, TORQUE_LIMITS).reshape(-1, 1)
                for i, pub in enumerate(self.joint_force_pubs):
                    force_msg = Float64()
                    force_msg.data = float(input_tq[i, 0])
                    pub.publish(force_msg)
                self.joint_efforts = input_tq.flatten()
                return

        # rl_deploy is in active control (StandUp, RLControl, etc.)
        effective_vel = self.vel_cmd.copy()
        effective_vel[[3, 7, 11, 15], 0] *= self.wheel_vel_scale
        self.effective_vel_cmd = effective_vel

        effective_kd = self.kd_cmd.copy()
        effective_kd[[3, 7, 11, 15], 0] *= self.wheel_kd_scale
        self.effective_kd_cmd = effective_kd

        input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            effective_kd * (effective_vel - dq) +
            self.tau_ff
        )
        # Clamp to physical motor limits so Gazebo ODE stays stable
        input_tq = np.clip(input_tq.flatten(), -TORQUE_LIMITS, TORQUE_LIMITS).reshape(-1, 1)

        # Check for NaN or Inf values in inputs
        if np.any(np.isnan(self.pos_cmd)) or np.any(np.isinf(self.pos_cmd)):
            self.get_logger().warn(f"Invalid pos_cmd detected: {self.pos_cmd.flatten()[:4]}")
            self.pos_cmd = np.nan_to_num(self.pos_cmd, nan=0.0, posinf=0.0, neginf=0.0)

        if np.any(np.isnan(self.kp_cmd)) or np.any(np.isinf(self.kp_cmd)):
            self.get_logger().warn(f"Invalid kp_cmd detected: {self.kp_cmd.flatten()[:4]}")
            self.kp_cmd = np.nan_to_num(self.kp_cmd, nan=0.0, posinf=0.0, neginf=0.0)

        # Check for NaN or Inf values in output
        if np.any(np.isnan(input_tq)) or np.any(np.isinf(input_tq)):
            self.get_logger().error(
                f"Invalid torque detected! "
                f"pos_cmd[0]={self.pos_cmd[0,0]:.3f}, q[0]={q[0,0]:.3f}, "
                f"kp[0]={self.kp_cmd[0,0]:.3f}, kd[0]={self.kd_cmd[0,0]:.3f}"
            )
            input_tq = np.nan_to_num(input_tq, nan=0.0, posinf=0.0, neginf=0.0)

        # Publish individual force commands for each joint
        for i, pub in enumerate(self.joint_force_pubs):
            force_msg = Float64()
            force_msg.data = float(input_tq[i, 0])
            pub.publish(force_msg)

        # Store computed torques for state publishing
        self.joint_efforts = input_tq.flatten()

    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q.w, q.x, q.y, q.z

        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _publish_robot_state(self):
        """Publish robot state in custom message format (200 Hz)"""
        self.timestamp = self.get_clock().now().nanoseconds / 1e9

        # ----- IMU -----
        rpy_rad = self.quaternion_to_euler(self.imu_orientation)
        rpy_deg = [angle * (180.0 / np.pi) for angle in rpy_rad]

        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(self.imu_angular_velocity[0])
        imu_msg.data.omega_y = float(self.imu_angular_velocity[1])
        imu_msg.data.omega_z = float(self.imu_angular_velocity[2])
        imu_msg.data.acc_x = float(self.imu_linear_acceleration[0])
        imu_msg.data.acc_y = float(self.imu_linear_acceleration[1])
        imu_msg.data.acc_z = float(self.imu_linear_acceleration[2])
        self.imu_pub.publish(imu_msg)

        # ----- Joints -----
        q = self.joint_positions
        dq = self.joint_velocities
        tau = self.joint_efforts

        # Convert raw to published: published = (raw - offset_rad) * dir
        pub_pos = (q - POS_OFFSET_RAD) * JOINT_DIR
        pub_vel = dq * JOINT_DIR
        pub_tau = tau * JOINT_DIR

        # Sanitize values before publishing
        pub_pos = np.nan_to_num(pub_pos, nan=0.0, posinf=0.0, neginf=0.0)
        pub_vel = np.nan_to_num(pub_vel, nan=0.0, posinf=0.0, neginf=0.0)
        pub_tau = np.nan_to_num(pub_tau, nan=0.0, posinf=0.0, neginf=0.0)

        joints_msg = JointsData()
        joints_msg.header = MetaType()
        joints_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(self.dof_num)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]
            joint.name = [32, 32, 32, 32]  # ASCII spaces as uint8 array
            joint.data_id = 0
            joint.status_word = 1  # Normal
            joint.position = float(pub_pos[i])
            joint.torque = float(pub_tau[i])
            joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0  # Dummy normal temp
            joint.driver_temp = 45.0  # Dummy normal temp
        self.joints_pub.publish(joints_msg)


def main(args=None):
    rclpy.init(args=args)
    controller_node = GazeboControllerNode()

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
