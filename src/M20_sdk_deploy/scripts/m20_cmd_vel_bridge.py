#!/usr/bin/env python3
"""
m20_cmd_vel_bridge.py — Bridges CMU autonomy stack cmd_vel to the M20 robot interface.

The CMU pathFollower publishes geometry_msgs/TwistStamped on /cmd_vel.
The M20 ROS2CmdVelInterface expects geometry_msgs/Twist on /M20/cmd_vel.

This node strips the header, clamps velocities to M20 hardware limits, and republishes.

Subscribes:
  /cmd_vel        (geometry_msgs/TwistStamped)  — from CMU pathFollower

Publishes:
  /M20/cmd_vel    (geometry_msgs/Twist)          — to M20 ROS2CmdVelInterface

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

        self._max_lx  = self.get_parameter('max_linear_x').get_parameter_value().double_value
        self._max_ly  = self.get_parameter('max_linear_y').get_parameter_value().double_value
        self._max_az  = self.get_parameter('max_angular_z').get_parameter_value().double_value

        self._pub = self.create_publisher(Twist, '/M20/cmd_vel', 10)
        self._sub = self.create_subscription(
            TwistStamped, '/cmd_vel', self._callback, 10)

        self.get_logger().info(
            f"m20_cmd_vel_bridge ready: /cmd_vel (TwistStamped) → /M20/cmd_vel (Twist)\n"
            f"  limits: linear_x={self._max_lx} m/s, linear_y={self._max_ly} m/s, "
            f"angular_z={self._max_az} rad/s")

    def _callback(self, msg: TwistStamped):
        out = Twist()
        out.linear.x  = _clamp(msg.twist.linear.x,  self._max_lx)
        out.linear.y  = _clamp(msg.twist.linear.y,  self._max_ly)
        out.angular.z = _clamp(msg.twist.angular.z, self._max_az)
        self._pub.publish(out)


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
