#!/usr/bin/env python3
"""
m20_imu_adapter.py — Converts M20 proprietary IMU data to standard ROS2 IMU messages.

Subscribes:
  /M20/IMU_DATA  (drdds/ImuData)  — raw IMU from DDS hardware interface

Publishes:
  /imu/data  (sensor_msgs/Imu)   — standard IMU consumed by Point-LIO SLAM

Parameters (ros2 params):
  imu_frame_id  (str, "imu_sensor")  — TF frame for the output IMU message
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from drdds.msg import ImuData


def _euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float):
    """ZYX Euler angles (radians) → quaternion (x, y, z, w)."""
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


class M20ImuAdapter(Node):
    def __init__(self):
        super().__init__('m20_imu_adapter')

        self.declare_parameter('imu_frame_id', 'imu_sensor')
        self._frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value

        self._pub = self.create_publisher(Imu, '/imu/data', 10)
        self._sub = self.create_subscription(
            ImuData, '/M20/IMU_DATA', self._callback, 10)

        self.get_logger().info(
            f"m20_imu_adapter ready: /M20/IMU_DATA → /imu/data (frame: {self._frame_id})")

    def _callback(self, msg: ImuData):
        # drdds/ImuDataValue stores Euler angles in degrees
        roll_rad  = math.radians(msg.data.roll)
        pitch_rad = math.radians(msg.data.pitch)
        yaw_rad   = math.radians(msg.data.yaw)
        qx, qy, qz, qw = _euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        imu_out = Imu()
        imu_out.header.stamp    = msg.header.stamp
        imu_out.header.frame_id = self._frame_id

        imu_out.orientation.x = qx
        imu_out.orientation.y = qy
        imu_out.orientation.z = qz
        imu_out.orientation.w = qw
        # Covariance unknown — signal this with -1 in the first element
        imu_out.orientation_covariance[0] = -1.0

        imu_out.angular_velocity.x = msg.data.omega_x
        imu_out.angular_velocity.y = msg.data.omega_y
        imu_out.angular_velocity.z = msg.data.omega_z
        imu_out.angular_velocity_covariance[0] = 0.01
        imu_out.angular_velocity_covariance[4] = 0.01
        imu_out.angular_velocity_covariance[8] = 0.01

        imu_out.linear_acceleration.x = msg.data.acc_x
        imu_out.linear_acceleration.y = msg.data.acc_y
        imu_out.linear_acceleration.z = msg.data.acc_z
        imu_out.linear_acceleration_covariance[0] = 0.1
        imu_out.linear_acceleration_covariance[4] = 0.1
        imu_out.linear_acceleration_covariance[8] = 0.1

        self._pub.publish(imu_out)


def main(args=None):
    rclpy.init(args=args)
    node = M20ImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
