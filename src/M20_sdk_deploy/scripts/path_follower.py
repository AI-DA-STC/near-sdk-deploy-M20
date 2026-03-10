#!/usr/bin/env python3
"""
path_follower.py — Pure-pursuit autonomous path follower for the M20 robot.

Subscribes:
  /route_server/path  (nav_msgs/Path)              — dense planned path
  /amcl_pose          (PoseWithCovarianceStamped)  — robot's current pose

Publishes:
  /M20/cmd_vel  (geometry_msgs/Twist)  — velocity commands consumed by
                                         ROS2CmdVelInterface in the C++ state machine

Parameters (ros2 params):
  max_speed           (float, 0.5)   — forward speed cap [m/s]
  angular_gain        (float, 1.0)   — yaw rate = angular_gain * heading_error [rad/s per rad]
  lookahead_distance  (float, 1.5)   — pure-pursuit lookahead [m]
  goal_tolerance      (float, 0.4)   — stop when within this distance of the final pose [m]
  control_frequency   (float, 10.0)  — control loop rate [Hz]
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped


def _quat_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PathFollower(Node):
    def __init__(self) -> None:
        super().__init__("path_follower")

        self.declare_parameter("max_speed",          0.5)
        self.declare_parameter("angular_gain",       1.0)
        self.declare_parameter("lookahead_distance", 1.5)
        self.declare_parameter("goal_tolerance",     0.4)
        self.declare_parameter("control_frequency",  10.0)

        self._max_speed    = self.get_parameter("max_speed").value
        self._ang_gain     = self.get_parameter("angular_gain").value
        self._lookahead    = self.get_parameter("lookahead_distance").value
        self._goal_tol     = self.get_parameter("goal_tolerance").value
        freq               = self.get_parameter("control_frequency").value

        self._path: list = []        # list of (x, y) from the latest Path msg
        self._robot_x: float | None = None
        self._robot_y: float | None = None
        self._robot_yaw: float | None = None
        self._goal_reached = False

        # Latch-compatible subscriber for the planned path
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/route_server/path",
                                 self._cb_path, latched_qos)

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self._cb_amcl, 10)

        self._cmd_pub = self.create_publisher(Twist, "/M20/cmd_vel", 10)

        self.create_timer(1.0 / freq, self._control_loop)

        self.get_logger().info(
            f"PathFollower ready — lookahead={self._lookahead} m, "
            f"max_speed={self._max_speed} m/s, goal_tol={self._goal_tol} m"
        )

    # ── subscribers ─────────────────────────────────────────────────────────

    def _cb_path(self, msg: Path) -> None:
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._goal_reached = False
        self.get_logger().info(f"New path received: {len(self._path)} poses")

    def _cb_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._robot_x   = msg.pose.pose.position.x
        self._robot_y   = msg.pose.pose.position.y
        self._robot_yaw = _quat_to_yaw(msg.pose.pose.orientation)

    # ── pure pursuit ────────────────────────────────────────────────────────

    def _find_lookahead(self, rx: float, ry: float) -> tuple[float, float] | None:
        """
        Walk along the path from the closest point and return the first point
        that is at least lookahead_distance away from the robot.
        Falls back to the goal if the remaining path is shorter than the lookahead.
        """
        if not self._path:
            return None

        # Find index of the closest point
        dists = [math.hypot(x - rx, y - ry) for x, y in self._path]
        closest_idx = int(min(range(len(dists)), key=lambda i: dists[i]))

        # Walk forward until lookahead distance is exceeded
        for i in range(closest_idx, len(self._path)):
            px, py = self._path[i]
            if math.hypot(px - rx, py - ry) >= self._lookahead:
                return px, py

        # Path is shorter than lookahead — return the goal
        return self._path[-1]

    def _control_loop(self) -> None:
        cmd = Twist()  # zero by default (safe stop)

        if not self._path:
            self._cmd_pub.publish(cmd)
            return

        if self._robot_x is None:
            return  # waiting for first AMCL pose

        if self._goal_reached:
            self._cmd_pub.publish(cmd)
            return

        rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

        # Check if goal is reached
        gx, gy = self._path[-1]
        if math.hypot(gx - rx, gy - ry) < self._goal_tol:
            self._goal_reached = True
            self._cmd_pub.publish(cmd)
            self.get_logger().info("Goal reached — stopping.")
            return

        target = self._find_lookahead(rx, ry)
        if target is None:
            self._cmd_pub.publish(cmd)
            return

        tx, ty = target

        # Heading error in robot body frame
        alpha = _wrap_angle(math.atan2(ty - ry, tx - rx) - ryaw)

        # Pure pursuit: forward speed decreases when turning sharply
        vx   = self._max_speed * max(0.0, math.cos(alpha))
        vyaw = self._ang_gain  * alpha

        cmd.linear.x  = vx
        cmd.angular.z = vyaw
        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"Pursuing ({tx:.2f},{ty:.2f})  alpha={math.degrees(alpha):.1f}°  "
            f"vx={vx:.2f}  vyaw={vyaw:.2f}"
        )


def main() -> None:
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
