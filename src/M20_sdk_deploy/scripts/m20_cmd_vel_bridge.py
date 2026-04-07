#!/usr/bin/env python3
"""
m20_cmd_vel_bridge.py — Bridges cmd_vel to the M20 robot interface.

Accepts both geometry_msgs/Twist (Nav2 controller_server) and
geometry_msgs/TwistStamped (CMU pathFollower) on /cmd_vel.
Clamps velocities to M20 hardware limits and republishes as
geometry_msgs/Twist on /M20/cmd_vel.

Subscribes:
  /cmd_vel            (geometry_msgs/Twist)          — from Nav2 controller_server
  /cmd_vel_stamped    (geometry_msgs/TwistStamped)   — from CMU pathFollower

Publishes:
  /M20/cmd_vel    (geometry_msgs/Twist)              — to M20 ROS2CmdVelInterface

Parameters (ros2 params):
  max_linear_x   (float, 0.5)   — forward/backward speed limit [m/s]
  max_linear_y   (float, 0.25)  — lateral speed limit [m/s]
  max_angular_z  (float, 1.0)   — yaw rate limit [rad/s]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class M20CmdVelBridge(Node):
    def __init__(self):
        super().__init__('m20_cmd_vel_bridge')

        self.declare_parameter('max_linear_x',  0.5)
        self.declare_parameter('max_linear_y',  0.25)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('cmd_timeout',   0.5)

        self._max_lx  = self.get_parameter('max_linear_x').get_parameter_value().double_value
        self._max_ly  = self.get_parameter('max_linear_y').get_parameter_value().double_value
        self._max_az  = self.get_parameter('max_angular_z').get_parameter_value().double_value
        self._cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value

        self._pub = self.create_publisher(Twist, '/M20/cmd_vel', 10)

        # Accept Twist from Nav2 controller_server
        self._sub_twist = self.create_subscription(
            Twist, '/cmd_vel', self._twist_cb, 10)

        # Accept TwistStamped from CMU pathFollower
        self._sub_stamped = self.create_subscription(
            TwistStamped, '/cmd_vel_stamped', self._stamped_cb, 10)

        # Watchdog: zero velocity if no command received within timeout
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(
            self._cmd_timeout, self._watchdog_cb
        )

        self.get_logger().info(
            f"m20_cmd_vel_bridge ready: /cmd_vel (Twist) + /cmd_vel_stamped (TwistStamped) -> /M20/cmd_vel\n"
            f"  limits: linear_x={self._max_lx} m/s, linear_y={self._max_ly} m/s, "
            f"angular_z={self._max_az} rad/s  |  watchdog: {self._cmd_timeout}s")

    def _publish_clamped(self, lx: float, ly: float, az: float):
        out = Twist()
        out.linear.x  = _clamp(lx, self._max_lx)
        out.linear.y  = _clamp(ly, self._max_ly)
        out.angular.z = _clamp(az, self._max_az)
        self._pub.publish(out)
        self._last_cmd_time = self.get_clock().now()

    def _twist_cb(self, msg: Twist):
        self._publish_clamped(msg.linear.x, msg.linear.y, msg.angular.z)

    def _stamped_cb(self, msg: TwistStamped):
        self._publish_clamped(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

    def _watchdog_cb(self):
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > self._cmd_timeout:
            self._pub.publish(Twist())  # zero velocity


def main(args=None):
    rclpy.init(args=args)
    node = M20CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
