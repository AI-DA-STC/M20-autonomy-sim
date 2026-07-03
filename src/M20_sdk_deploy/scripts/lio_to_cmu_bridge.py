#!/usr/bin/env python3
"""
Bridge Point-LIO output to CMU autonomy stack conventions.

Point-LIO publishes:
  /aft_mapped_to_init  (nav_msgs/Odometry, frame=camera_init, child=aft_mapped)
  /cloud_registered    (PointCloud2, frame=camera_init)

CMU stack expects:
  /state_estimation    (nav_msgs/Odometry, frame=map, child=sensor)
  /registered_scan     (PointCloud2, frame=map)
  TF: map -> sensor    (dynamic, from odometry)
  TF: camera_init -> map  (static identity, so CMU can use Point-LIO TFs too)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from geometry_msgs.msg import TransformStamped
import math
import struct
import tf2_ros


class LioCmuBridge(Node):
    def __init__(self):
        super().__init__('lio_cmu_bridge')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Static TF: camera_init == map (same world frame, different names)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'map'
        st.child_frame_id = 'camera_init'
        st.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(st)

        self.pub_odom = self.create_publisher(Odometry, '/state_estimation', 5)
        self.pub_scan = self.create_publisher(PointCloud2, '/registered_scan', 2)
        self.pub_overall_map = self.create_publisher(PointCloud2, '/overall_map', 2)

        self.declare_parameter('self_filter_enabled', True)
        self.declare_parameter('self_filter_min_x', -0.45)
        self.declare_parameter('self_filter_max_x', 0.55)
        self.declare_parameter('self_filter_min_y', -0.35)
        self.declare_parameter('self_filter_max_y', 0.35)
        self.declare_parameter('self_filter_min_z', -0.35)
        self.declare_parameter('self_filter_max_z', 0.45)
        self.declare_parameter('self_filter_min_range', 0.20)
        self.declare_parameter('self_filter_min_keep_points', 200)
        self.declare_parameter('self_filter_min_keep_ratio', 0.05)
        self.declare_parameter('publish_overall_map', True)
        self.declare_parameter('overall_map_voxel_size', 0.10)
        self.declare_parameter('overall_map_publish_every_n_scans', 5)
        self.declare_parameter('overall_map_max_points', 250000)

        self.self_filter_enabled = self.get_parameter('self_filter_enabled').value
        self.self_filter_min_x = float(self.get_parameter('self_filter_min_x').value)
        self.self_filter_max_x = float(self.get_parameter('self_filter_max_x').value)
        self.self_filter_min_y = float(self.get_parameter('self_filter_min_y').value)
        self.self_filter_max_y = float(self.get_parameter('self_filter_max_y').value)
        self.self_filter_min_z = float(self.get_parameter('self_filter_min_z').value)
        self.self_filter_max_z = float(self.get_parameter('self_filter_max_z').value)
        self.self_filter_min_range = float(self.get_parameter('self_filter_min_range').value)
        self.self_filter_min_keep_points = int(self.get_parameter('self_filter_min_keep_points').value)
        self.self_filter_min_keep_ratio = float(self.get_parameter('self_filter_min_keep_ratio').value)
        self.publish_overall_map = self.get_parameter('publish_overall_map').value
        self.overall_map_voxel_size = float(self.get_parameter('overall_map_voxel_size').value)
        self.overall_map_publish_every_n_scans = max(
            1, int(self.get_parameter('overall_map_publish_every_n_scans').value))
        self.overall_map_max_points = max(
            1, int(self.get_parameter('overall_map_max_points').value))

        self.latest_pose = None
        self.scan_count = 0
        self.overall_map = {}

        self.create_subscription(Odometry, '/aft_mapped_to_init', self._odom_cb, 5)
        self.create_subscription(PointCloud2, '/cloud_registered', self._scan_cb, 2)

        self.get_logger().info(
            'LIO->CMU bridge self filter: '
            f'enabled={self.self_filter_enabled}, '
            f'box=([{self.self_filter_min_x}, {self.self_filter_max_x}], '
            f'[{self.self_filter_min_y}, {self.self_filter_max_y}], '
            f'[{self.self_filter_min_z}, {self.self_filter_max_z}]), '
            f'min_range={self.self_filter_min_range}, '
            f'fallback_keep={self.self_filter_min_keep_points}/'
            f'{self.self_filter_min_keep_ratio:.2f}, '
            f'overall_map={self.publish_overall_map}, '
            f'overall_voxel={self.overall_map_voxel_size}'
        )

    def _odom_cb(self, msg: Odometry):
        # Republish with CMU frame names
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'sensor'
        self.pub_odom.publish(msg)
        self.latest_pose = msg.pose.pose

        # Broadcast TF map -> sensor
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'sensor'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

    def _scan_cb(self, msg: PointCloud2):
        msg.header.frame_id = 'map'
        if self.self_filter_enabled and self.latest_pose is not None:
            msg = self._filter_self_points(msg)
        self.pub_scan.publish(msg)
        if self.publish_overall_map:
            self._update_overall_map(msg)

    @staticmethod
    def _quat_conj_rotate(q, x, y, z):
        # Rotate vector by inverse unit quaternion q^-1.
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # q_conj * v * q, expanded.
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = qx * x + qy * y + qz * z
        rx = ix * qw + iw * qx + iy * qz - iz * qy
        ry = iy * qw + iw * qy + iz * qx - ix * qz
        rz = iz * qw + iw * qz + ix * qy - iy * qx
        return rx, ry, rz

    def _filter_self_points(self, msg: PointCloud2):
        field_names = [field.name for field in msg.fields]
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            return msg

        offsets = {field.name: field.offset for field in msg.fields}
        x_offset = offsets['x']
        y_offset = offsets['y']
        z_offset = offsets['z']
        unpack_float = struct.Struct('>f' if msg.is_bigendian else '<f').unpack_from
        pose = self.latest_pose
        tx = pose.position.x
        ty = pose.position.y
        tz = pose.position.z
        q = pose.orientation

        kept_data = bytearray()
        total = 0
        removed = 0
        point_step = msg.point_step
        data = msg.data
        point_count = len(data) // point_step if point_step > 0 else 0
        for i in range(point_count):
            base = i * point_step
            total += 1
            px = unpack_float(data, base + x_offset)[0]
            py = unpack_float(data, base + y_offset)[0]
            pz = unpack_float(data, base + z_offset)[0]
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                removed += 1
                continue
            lx, ly, lz = self._quat_conj_rotate(q, px - tx, py - ty, pz - tz)
            local_range = (lx * lx + ly * ly + lz * lz) ** 0.5

            in_self_box = (
                self.self_filter_min_x <= lx <= self.self_filter_max_x and
                self.self_filter_min_y <= ly <= self.self_filter_max_y and
                self.self_filter_min_z <= lz <= self.self_filter_max_z
            )
            too_close = local_range < self.self_filter_min_range
            if in_self_box or too_close:
                removed += 1
                continue
            kept_data.extend(data[base:base + point_step])

        min_keep = max(self.self_filter_min_keep_points,
                       int(total * self.self_filter_min_keep_ratio))
        kept_count = len(kept_data) // point_step if point_step > 0 else 0
        if total > 0 and kept_count < min_keep:
            self.scan_count += 1
            if self.scan_count == 1 or self.scan_count % 20 == 0:
                self.get_logger().warn(
                    f'self-filter fallback on scan #{self.scan_count}: '
                    f'{total} -> {kept_count} points would remain; publishing original'
                )
            return msg

        filtered = PointCloud2()
        filtered.header = msg.header
        filtered.height = 1
        filtered.width = kept_count
        filtered.fields = msg.fields
        filtered.is_bigendian = msg.is_bigendian
        filtered.point_step = msg.point_step
        filtered.row_step = msg.point_step * kept_count
        filtered.data = bytes(kept_data)
        filtered.is_dense = msg.is_dense
        self.scan_count += 1
        if self.scan_count == 1 or self.scan_count % 20 == 0:
            self.get_logger().info(
                f'self-filtered registered_scan #{self.scan_count}: '
                f'{total} -> {kept_count} points, removed={removed}'
            )
        return filtered

    def _update_overall_map(self, msg: PointCloud2):
        field_names = [field.name for field in msg.fields]
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            return

        offsets = {field.name: field.offset for field in msg.fields}
        x_offset = offsets['x']
        y_offset = offsets['y']
        z_offset = offsets['z']
        intensity_offset = offsets.get('intensity')
        unpack_float = struct.Struct('>f' if msg.is_bigendian else '<f').unpack_from
        point_step = msg.point_step
        if point_step <= 0:
            return

        inv_voxel = 1.0 / self.overall_map_voxel_size
        point_count = len(msg.data) // point_step
        for i in range(point_count):
            base = i * point_step
            px = unpack_float(msg.data, base + x_offset)[0]
            py = unpack_float(msg.data, base + y_offset)[0]
            pz = unpack_float(msg.data, base + z_offset)[0]
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue
            intensity = 0.0
            if intensity_offset is not None:
                intensity = unpack_float(msg.data, base + intensity_offset)[0]
                if not math.isfinite(intensity):
                    intensity = 0.0
            key = (
                int(math.floor(px * inv_voxel)),
                int(math.floor(py * inv_voxel)),
                int(math.floor(pz * inv_voxel)),
            )
            self.overall_map[key] = (px, py, pz, intensity)

        # Bound memory for long sessions. Dict insertion order keeps recent voxels.
        overflow = len(self.overall_map) - self.overall_map_max_points
        if overflow > 0:
            for key in list(self.overall_map.keys())[:overflow]:
                del self.overall_map[key]

        if self.scan_count % self.overall_map_publish_every_n_scans == 0:
            self._publish_overall_map(msg.header)

    def _publish_overall_map(self, header):
        points = list(self.overall_map.values())
        cloud = PointCloud2()
        cloud.header = header
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        pack = struct.Struct('<ffff').pack
        data = bytearray()
        for px, py, pz, intensity in points:
            data.extend(pack(px, py, pz, intensity))
        cloud.data = bytes(data)
        self.pub_overall_map.publish(cloud)
        if self.scan_count == 1 or self.scan_count % 50 == 0:
            self.get_logger().info(
                f'published exploration overall_map: {cloud.width} voxels'
            )


def main():
    rclpy.init()
    node = LioCmuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
