#!/usr/bin/env python3
"""
velodyne_timestamp_injector.py

Subscribes to /M20/LIDAR/VELODYNE (organized PointCloud2 without per-point
timestamps) and republishes on /velodyne_points with synthetic per-point
'time' fields computed from column index (azimuth position).

This enables Point-LIO's point-by-point EKF updates which require
monotonically increasing per-point timestamps within each scan.

The Gazebo Velodyne message has a 4-byte gap at offset 20 (between
intensity@16 and ring@24), which is exactly where velodyne_ros::Point
expects the 'time' field. We inject timestamps there without changing
point_step.

Subscribes:  /M20/LIDAR/VELODYNE  (sensor_msgs/PointCloud2)
Publishes:   /velodyne_points      (sensor_msgs/PointCloud2, with 'time' field)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class VelodyneTimestampInjector(Node):
    def __init__(self):
        super().__init__('velodyne_timestamp_injector')

        self.declare_parameter('scan_rate', 10.0)
        self._scan_rate = (
            self.get_parameter('scan_rate')
            .get_parameter_value().double_value
        )

        self._pub = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self._sub = self.create_subscription(
            PointCloud2, '/M20/LIDAR/VELODYNE', self._callback, 10)

        self.get_logger().info(
            f'Velodyne timestamp injector: '
            f'/M20/LIDAR/VELODYNE -> /velodyne_points '
            f'(scan_rate={self._scan_rate} Hz)')

    def _callback(self, msg: PointCloud2):
        height = msg.height       # 32 rings
        width = msg.width         # 2187 azimuth steps
        point_step = msg.point_step  # 32

        # Reshape data for efficient manipulation
        data = np.frombuffer(msg.data, dtype=np.uint8).copy()
        data = data.reshape(height, width, point_step)

        # Per-column time offset in seconds: [0, 1/scan_rate)
        # All 32 rings at the same azimuth column fire simultaneously
        time_offsets = (
            np.arange(width, dtype=np.float32) / width / self._scan_rate
        )

        # Write time (float32) into the 4-byte gap at offset 20
        time_bytes = time_offsets.view(np.uint8).reshape(width, 4)
        data[:, :, 20:24] = time_bytes[np.newaxis, :, :]

        # Add 'time' field descriptor
        new_fields = list(msg.fields)
        new_fields.append(PointField(
            name='time', offset=20,
            datatype=PointField.FLOAT32, count=1))

        # Build output message
        out = PointCloud2()
        out.header = msg.header
        out.height = height
        out.width = width
        out.fields = new_fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = point_step   # unchanged — used existing gap
        out.row_step = msg.row_step
        out.data = data.tobytes()
        out.is_dense = msg.is_dense

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneTimestampInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
