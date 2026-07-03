#!/usr/bin/python3
"""
Merges front and rear LiDAR point clouds into a single topic for GLIM.
Transforms rear LiDAR points to front LiDAR frame before merging.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')

        # Get robot name from parameter
        self.declare_parameter('robot_name', 'M20')
        self.declare_parameter('front_x', 0.38)
        self.declare_parameter('rear_x', -0.38)
        robot_name = self.get_parameter('robot_name').value
        front_x = float(self.get_parameter('front_x').value)
        rear_x = float(self.get_parameter('rear_x').value)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.front_sub = self.create_subscription(
            PointCloud2,
            f'/{robot_name}/LIDAR/FRONT',
            self.front_callback,
            sensor_qos
        )
        self.rear_sub = self.create_subscription(
            PointCloud2,
            f'/{robot_name}/LIDAR/REAR',
            self.rear_callback,
            sensor_qos
        )

        # Publisher
        self.merged_pub = self.create_publisher(
            PointCloud2,
            f'/{robot_name}/LIDAR/MERGED',
            sensor_qos
        )

        # Storage for synchronization
        self.front_cloud = None
        self.rear_cloud = None
        # LiDARs may have up to 200ms timestamp difference, allow 250ms tolerance
        self.sync_tolerance = 0.25

        # Debug counters
        self.merge_count = 0
        self.sync_fail_count = 0

        # Rear LiDAR transform into the front LiDAR frame. The rear link has
        # yaw=pi relative to base_link, so x/y are flipped before translation.
        self.rear_to_front_offset = np.array([rear_x - front_x, 0.0, 0.0])

        self.get_logger().info(
            f'PointCloud merger initialized for {robot_name}, '
            f'rear_to_front_offset={self.rear_to_front_offset.tolist()}'
        )

    def front_callback(self, msg):
        self.front_cloud = msg
        self.try_merge()

    def rear_callback(self, msg):
        self.rear_cloud = msg
        self.try_merge()

    def try_merge(self):
        if self.front_cloud is None or self.rear_cloud is None:
            return

        # Check timestamp synchronization
        front_time = self.front_cloud.header.stamp.sec + \
                     self.front_cloud.header.stamp.nanosec * 1e-9
        rear_time = self.rear_cloud.header.stamp.sec + \
                    self.rear_cloud.header.stamp.nanosec * 1e-9

        time_diff = abs(front_time - rear_time)
        if time_diff > self.sync_tolerance:
            self.sync_fail_count += 1
            if self.sync_fail_count % 50 == 1:  # Log every 50 failures
                self.get_logger().warn(
                    f'Sync failed: front={front_time:.3f}s rear={rear_time:.3f}s '
                    f'diff={time_diff:.3f}s (tolerance={self.sync_tolerance}s)'
                )
            return

        # Extract points
        front_points = list(pc2.read_points(self.front_cloud,
                                            field_names=('x', 'y', 'z'),
                                            skip_nans=True))
        rear_points = list(pc2.read_points(self.rear_cloud,
                                           field_names=('x', 'y', 'z'),
                                           skip_nans=True))

        # Transform rear points (180 deg rotation + translation)
        transformed_rear = []
        for p in rear_points:
            # Rotate 180 deg around Z and translate
            x_new = -p[0] + self.rear_to_front_offset[0]
            y_new = -p[1] + self.rear_to_front_offset[1]
            z_new = p[2] + self.rear_to_front_offset[2]
            transformed_rear.append((x_new, y_new, z_new))

        # Merge
        all_points = front_points + transformed_rear

        # Create merged message - use front cloud timestamp (simulation time)
        merged_msg = pc2.create_cloud_xyz32(self.front_cloud.header, all_points)
        self.merged_pub.publish(merged_msg)

        self.merge_count += 1
        if self.merge_count % 100 == 1:  # Log every 100 successful merges
            self.get_logger().info(
                f'Merged #{self.merge_count}: {len(front_points)}+{len(transformed_rear)}='
                f'{len(all_points)} points'
            )

        # Clear for next sync
        self.front_cloud = None
        self.rear_cloud = None


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
