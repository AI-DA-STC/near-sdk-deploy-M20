#!/usr/bin/env python3
"""
Absolute Pose Error (APE) evaluator — M20 AMCL + EKF localization.

Ground truth : /gz/world_poses  (tf2_msgs/TFMessage bridged from
               /world/Edifice/dynamic_pose/info, child_frame_id == "M20")
Estimated    : TF lookup  map -> base_link  (AMCL + EKF output)

APE is computed in *relative* coordinates, i.e. displacement from the
initial pose in each frame, so the absolute world-vs-map offset cancels.

Publishes (Float64):
    /M20/ape/translation_m   — Euclidean XY error [m]
    /M20/ape/rotation_deg    — absolute yaw error  [deg]

Writes CSV → /tmp/m20_ape_<YYYYMMDD_HHMMSS>.csv
"""

import csv
import datetime
import math

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node

import tf2_ros
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64


# ── helpers ──────────────────────────────────────────────────────────────────

def _quat_to_yaw(q) -> float:
    """Extract yaw (rotation about Z) from a quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _yaw_diff_deg(a_rad: float, b_rad: float) -> float:
    """Absolute angular difference in degrees, wrapped to [0°, 180°]."""
    d = abs(math.degrees(a_rad - b_rad)) % 360.0
    return d if d <= 180.0 else 360.0 - d


def _rotate_2d(x: float, y: float, theta: float):
    """Rotate point (x, y) by angle theta."""
    c, s = math.cos(theta), math.sin(theta)
    return c * x - s * y, s * x + c * y


# ── node ─────────────────────────────────────────────────────────────────────

class PoseAPEEvaluator(Node):

    # child_frame_id values ros_gz_bridge may use for the M20 model pose
    _GT_CANDIDATES = ("M20", "M20::base_link", "M20/base_link")

    def __init__(self):
        super().__init__("pose_ape_evaluator")

        # TF listener for estimated pose (map -> base_link)
        self._tf_buf = tf2_ros.Buffer()
        self._tf_sub = tf2_ros.TransformListener(self._tf_buf, self)

        # Publishers
        self._pub_trans = self.create_publisher(Float64, "/M20/ape/translation_m", 10)
        self._pub_rot   = self.create_publisher(Float64, "/M20/ape/rotation_deg",  10)

        # Ground-truth subscription (ros_gz_bridge remaps dynamic_pose/info here)
        self.create_subscription(
            TFMessage,
            "/gz/world_poses",
            self._gt_callback,
            10,
        )

        # State
        self._gt_frame: str | None = None  # which candidate we confirmed
        self._gt0  = None                  # (x, y, yaw) GT at t=0  in world frame
        self._est0 = None                  # (x, y, yaw) Est at t=0 in map frame

        # CSV log
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = f"/tmp/m20_ape_{ts}.csv"
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_wr   = csv.writer(self._csv_file)
        self._csv_wr.writerow([
            "sim_time_s",
            "gt_rel_x_m", "gt_rel_y_m", "gt_rel_yaw_deg",
            "est_rel_x_m", "est_rel_y_m", "est_rel_yaw_deg",
            "ape_trans_m", "ape_rot_deg",
        ])
        self.get_logger().info(f"APE log → {self._csv_path}")

    # ── ground-truth callback ─────────────────────────────────────────────────

    def _gt_callback(self, msg: TFMessage):
        # On first call, find which frame ID Gazebo uses for the M20 model
        if self._gt_frame is None:
            found = {tf.child_frame_id for tf in msg.transforms}
            for cand in self._GT_CANDIDATES:
                if cand in found:
                    self._gt_frame = cand
                    self.get_logger().info(f"Ground-truth frame confirmed: '{cand}'")
                    break
            else:
                self.get_logger().warn(
                    f"No M20 frame in /gz/world_poses yet. "
                    f"Received frames: {sorted(found)}"
                )
                return

        # Extract M20 model transform (world frame)
        gt_tf = next(
            (tf for tf in msg.transforms if tf.child_frame_id == self._gt_frame),
            None,
        )
        if gt_tf is None:
            return

        t  = gt_tf.transform.translation
        r  = gt_tf.transform.rotation
        gx, gy, gyaw = t.x, t.y, _quat_to_yaw(r)

        # Lookup estimated pose: map -> base_link
        try:
            est_tf = self._tf_buf.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return  # AMCL not ready yet

        et = est_tf.transform.translation
        er = est_tf.transform.rotation
        ex, ey, eyaw = et.x, et.y, _quat_to_yaw(er)

        # Use the EKF/AMCL TF stamp for sim time — the dynamic_pose/info bridge zeroes its own.
        s = est_tf.header.stamp
        sim_t = s.sec + s.nanosec * 1e-9

        # ── frame alignment at t=0 ────────────────────────────────────────────
        # Record initial poses to compute relative displacements.
        # This removes the world-frame vs map-frame absolute offset.
        if self._gt0 is None:
            self._gt0  = (gx, gy, gyaw)
            self._est0 = (ex, ey, eyaw)
            self.get_logger().info(
                f"Origins aligned:\n"
                f"  GT  world  ({gx:.3f} m, {gy:.3f} m, {math.degrees(gyaw):.1f}°)\n"
                f"  Est map    ({ex:.3f} m, {ey:.3f} m, {math.degrees(eyaw):.1f}°)"
            )

        # ── relative displacement from t=0 ────────────────────────────────────
        gx0, gy0, gyaw0 = self._gt0
        gt_rel_x, gt_rel_y = _rotate_2d(gx - gx0, gy - gy0, -gyaw0)
        gt_rel_yaw = gyaw - gyaw0

        ex0, ey0, eyaw0 = self._est0
        est_rel_x, est_rel_y = _rotate_2d(ex - ex0, ey - ey0, -eyaw0)
        est_rel_yaw = eyaw - eyaw0

        # ── APE ───────────────────────────────────────────────────────────────
        ape_trans = math.hypot(gt_rel_x - est_rel_x, gt_rel_y - est_rel_y)
        ape_rot   = _yaw_diff_deg(gt_rel_yaw, est_rel_yaw)

        # Publish
        self._pub_trans.publish(Float64(data=ape_trans))
        self._pub_rot.publish(Float64(data=ape_rot))

        # CSV
        self._csv_wr.writerow([
            f"{sim_t:.3f}",
            f"{gt_rel_x:.4f}", f"{gt_rel_y:.4f}", f"{math.degrees(gt_rel_yaw):.2f}",
            f"{est_rel_x:.4f}", f"{est_rel_y:.4f}", f"{math.degrees(est_rel_yaw):.2f}",
            f"{ape_trans:.4f}", f"{ape_rot:.2f}",
        ])
        self._csv_file.flush()

        self.get_logger().info(
            f"APE  trans={ape_trans:.3f} m  rot={ape_rot:.1f}°  "
            f"(t={sim_t:.1f} s)",
            throttle_duration_sec=1.0,
        )

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(f"APE log saved → {self._csv_path}")
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = PoseAPEEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
