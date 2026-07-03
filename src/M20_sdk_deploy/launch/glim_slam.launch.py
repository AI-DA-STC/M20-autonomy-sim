"""
GLIM SLAM Launch File for M20 Multi-Robot

Launches GLIM nodes for each robot with appropriate configuration.
Includes point cloud mergers and static TF publishers.

Usage:
  ros2 launch rl_deploy glim_slam.launch.py

  # Or for single robot:
  ros2 launch rl_deploy glim_slam.launch.py robots:="['M20_A']"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os

CONFIG_BASE_PATH = "/home/cjy/deeprobotics_ws/src/M20_sdk_deploy/config/glim"


def create_static_tf_publishers(robot_name):
    """Create static transform publishers for LiDAR frames"""
    return [
        # Front LiDAR TF: base_link -> front_lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_front_lidar_tf',
            arguments=[
                '0.32028', '0', '-0.013', '0', '0', '0',
                f'{robot_name}/base_link', f'{robot_name}/front_lidar'
            ],
        ),
        # Rear LiDAR TF: base_link -> rear_lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_rear_lidar_tf',
            arguments=[
                '-0.32028', '0', '-0.013', '0', '0', '3.14159',
                f'{robot_name}/base_link', f'{robot_name}/rear_lidar'
            ],
        ),
        # IMU TF: base_link -> imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_imu_tf',
            arguments=[
                '0.0632', '-0.0268', '-0.0435', '0', '0', '0',
                f'{robot_name}/base_link', f'{robot_name}/imu'
            ],
        ),
    ]


def create_glim_nodes(robot_name):
    """Create GLIM-related nodes for a single robot"""
    config_path = os.path.join(CONFIG_BASE_PATH, robot_name)

    nodes = []

    # Static TF publishers
    nodes.extend(create_static_tf_publishers(robot_name))

    # Point cloud merger
    nodes.append(
        Node(
            package='rl_deploy',
            executable='pointcloud_merger.py',
            name=f'{robot_name}_pointcloud_merger',
            output='screen',
            parameters=[{'robot_name': robot_name}],
        )
    )

    # GLIM ROS node - topics are configured in config_ros.json
    nodes.append(
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name=f'{robot_name}_glim_slam',
            output='screen',
            parameters=[{'config_path': config_path}],
        )
    )

    return nodes


def generate_launch_description():
    robots = ['M20_A', 'M20_B']

    # Create nodes for all robots with staggered start
    robot_actions = []
    for i, robot in enumerate(robots):
        # Stagger GLIM node startup to avoid GPU resource contention
        delay = i * 5.0  # M20_A at 0s, M20_B at 5s

        robot_actions.append(
            TimerAction(
                period=delay,
                actions=create_glim_nodes(robot)
            )
        )

    return LaunchDescription(robot_actions)
