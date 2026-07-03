#!/usr/bin/env python3
"""Add Velodyne-style ring/time fields to Gazebo PointCloud2 messages."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudLioAdapter(Node):
    def __init__(self):
        super().__init__('pointcloud_lio_adapter')

        self.declare_parameter('input_topic', '/M20/LIDAR/FRONT')
        self.declare_parameter('output_topic', '/M20/LIDAR/FRONT_LIO')
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('default_ring_count', 16)
        self.declare_parameter('horizontal_samples', 2048)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.scan_rate = float(self.get_parameter('scan_rate').value)
        self.default_ring_count = int(self.get_parameter('default_ring_count').value)
        self.horizontal_samples = int(self.get_parameter('horizontal_samples').value)

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, input_qos
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, output_qos)
        self.count = 0

        self.get_logger().info(
            f'PointCloud LIO adapter: {self.input_topic} -> {self.output_topic}'
        )

    def cloud_callback(self, msg: PointCloud2):
        field_names = [field.name for field in msg.fields]
        has_intensity = 'intensity' in field_names
        read_fields = ('x', 'y', 'z', 'intensity') if has_intensity else ('x', 'y', 'z')

        width = max(1, int(msg.width))
        height = max(1, int(msg.height))
        ring_count = height if height > 1 else self.default_ring_count
        horizontal_samples = (
            width if height > 1
            else self.horizontal_samples if self.horizontal_samples > 0
            else max(1, width // max(1, ring_count))
        )
        scan_period = 1.0 / self.scan_rate if self.scan_rate > 0.0 else 0.1

        points = []
        min_range = float('inf')
        max_range = 0.0
        far_points = 0
        for idx, point in enumerate(pc2.read_points(msg, field_names=read_fields, skip_nans=False)):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            point_range = math.sqrt(x * x + y * y + z * z)
            min_range = min(min_range, point_range)
            max_range = max(max_range, point_range)
            if point_range > 0.5:
                far_points += 1

            intensity = float(point[3]) if has_intensity and len(point) > 3 else 0.0
            row = idx // width if height > 1 else idx // horizontal_samples
            col = idx % width if height > 1 else idx % horizontal_samples
            ring = min(ring_count - 1, row)
            rel_time = (col / max(1, horizontal_samples - 1)) * scan_period
            points.append((x, y, z, intensity, rel_time, int(ring)))

        # Match the memory layout expected by Point-LIO's PCL Velodyne point:
        # PCL_ADD_POINT4D pads x/y/z to 16 bytes, so intensity starts at 16.
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=24, datatype=PointField.UINT16, count=1),
        ]

        out = pc2.create_cloud(msg.header, fields, points)
        out.is_dense = True
        self.pub.publish(out)

        self.count += 1
        if self.count % 50 == 1:
            self.get_logger().info(
                f'Published LIO cloud #{self.count}: {len(points)} points, '
                f'{width}x{height}, rings={ring_count}, horizontal={horizontal_samples}, '
                f'range=[{min_range:.2f}, {max_range:.2f}]m, >0.5m={far_points}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudLioAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
