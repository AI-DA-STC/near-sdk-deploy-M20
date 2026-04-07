#!/usr/bin/env python3
"""
linear_orchestrator.py — Sequential Nav2 pipeline orchestrator for M20.

Receives the sparse route from route_server and drives it through the full
Nav2 pipeline in a linear sequence:

  1. ComputePathToPose  (planner_server  — costmap-aware dense path)
  2. SmoothPath         (smoother_server — Savitzky-Golay smoothing)
  3. FollowPath         (controller_server — DWB obstacle-avoidant tracking)

Recovery state machine on any pipeline failure:

  WAIT (2s) -> replan -> BACKUP (1m) -> replan -> CLEAR_COSTMAPS -> replan -> ABORT

Services:
  ~/stop   (std_srvs/Trigger)  — cancel goals, zero vel, JointDamping -> Idle
  ~/pause  (std_srvs/Trigger)  — cancel goals, zero vel, stay in RLControl

Subscribes:
  /route_server/path  (nav_msgs/Path)                — sparse/dense route
  /amcl_pose          (PoseWithCovarianceStamped)     — current robot pose

Action clients:
  compute_path_to_pose  (nav2_msgs/action/ComputePathToPose)
  smooth_path           (nav2_msgs/action/SmoothPath)
  follow_path           (nav2_msgs/action/FollowPath)
  backup                (nav2_msgs/action/BackUp)

Service clients:
  /route_server/get_plan                              (nav_msgs/srv/GetPlan)
  /global_costmap/clear_entirely_global_costmap       (nav2_msgs/srv/ClearEntireCostmap)
  /local_costmap/clear_entirely_local_costmap         (nav2_msgs/srv/ClearEntireCostmap)
"""

from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from nav2_msgs.action import ComputePathToPose, SmoothPath, FollowPath, BackUp
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class RecoveryLevel(IntEnum):
    """Graduated recovery: each failure escalates to the next level."""
    WAIT = 0
    BACKUP = 1
    CLEAR_COSTMAPS = 2
    ABORT = 3


class LinearOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("linear_orchestrator")

        # Parameters
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("replan_delay", 2.0)
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("smoother_id", "SavitzkyGolay")
        self.declare_parameter("backup_distance", 1.0)
        self.declare_parameter("backup_speed", 0.1)
        self.declare_parameter("backup_time_allowance", 15.0)
        self._max_retries = self.get_parameter("max_retries").value
        self._replan_delay = self.get_parameter("replan_delay").value
        self._planner_id = self.get_parameter("planner_id").value
        self._smoother_id = self.get_parameter("smoother_id").value
        self._backup_distance = self.get_parameter("backup_distance").value
        self._backup_speed = self.get_parameter("backup_speed").value
        self._backup_time_allowance = self.get_parameter("backup_time_allowance").value

        # Action clients for the Nav2 pipeline
        self._compute_path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose"
        )
        self._smooth_path_client = ActionClient(
            self, SmoothPath, "smooth_path"
        )
        self._follow_path_client = ActionClient(
            self, FollowPath, "follow_path"
        )
        self._backup_client = ActionClient(
            self, BackUp, "backup"
        )

        # Service clients for costmap clearing
        self._clear_global_costmap_cli = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        self._clear_local_costmap_cli = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )

        # Route server replan service
        self._get_plan_cli = self.create_client(
            GetPlan, "/route_server/get_plan"
        )

        # Publishers for stop/pause
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._target_mode_pub = self.create_publisher(Int32, "/M20/target_mode", 10)

        # Services: stop and pause
        self.create_service(Trigger, "~/stop", self._stop_cb)
        self.create_service(Trigger, "~/pause", self._pause_cb)

        # State
        self._follow_goal_handle = None
        self._recovery_level = RecoveryLevel.WAIT
        self._original_goal: PoseStamped | None = None
        self._amcl_pose: PoseWithCovarianceStamped | None = None
        self._replan_timer = None
        self._navigating = False

        # Subscribe to AMCL pose for replan start position
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10
        )

        # Subscribe to route_server path (transient-local for late joiners)
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            Path, "/route_server/path", self._path_cb, latched_qos
        )

        self.get_logger().info(
            "LinearOrchestrator ready — pipeline: "
            f"planner({self._planner_id}) -> smoother({self._smoother_id}) "
            f"-> controller | recovery: WAIT -> BACKUP -> CLEAR_COSTMAPS -> ABORT"
        )

    # ── Stop / Pause services ────────────────────────────────────────────────

    def _cancel_all_goals(self) -> None:
        """Cancel any active FollowPath goal and clear replan timer."""
        if self._replan_timer is not None:
            self._replan_timer.cancel()
            self._replan_timer = None

        if (
            self._follow_goal_handle is not None
            and self._follow_goal_handle.status == GoalStatus.STATUS_EXECUTING
        ):
            self._follow_goal_handle.cancel_goal_async()
            self._follow_goal_handle = None

        self._navigating = False

    def _publish_zero_velocity(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _stop_cb(self, request, response) -> Trigger.Response:
        """Full stop: cancel goals, zero velocity, transition to JointDamping -> Idle."""
        self.get_logger().info("STOP requested — cancelling navigation, entering JointDamping.")
        self._cancel_all_goals()
        self._publish_zero_velocity()

        # Publish target_mode=2 (JointDamping) — robot will damp for 3s then go Idle
        mode_msg = Int32()
        mode_msg.data = 2
        self._target_mode_pub.publish(mode_msg)

        self._recovery_level = RecoveryLevel.WAIT
        response.success = True
        response.message = "Navigation stopped. Robot entering JointDamping -> Idle."
        return response

    def _pause_cb(self, request, response) -> Trigger.Response:
        """Pause: cancel goals, zero velocity, stay in RLControl for manual override."""
        self.get_logger().info("PAUSE requested — cancelling navigation, manual control available.")
        self._cancel_all_goals()
        self._publish_zero_velocity()

        self._recovery_level = RecoveryLevel.WAIT
        response.success = True
        response.message = "Navigation paused. Manual control available via /M20/cmd_vel."
        return response

    # ── AMCL pose cache ──────────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl_pose = msg

    # ── Path received from route_server ──────────────────────────────────────

    def _path_cb(self, msg: Path) -> None:
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path from route_server, ignoring.")
            return

        # Reset recovery state for new user goal
        self._recovery_level = RecoveryLevel.WAIT
        self._original_goal = msg.poses[-1]
        self._navigating = True

        start = msg.poses[0]
        goal = msg.poses[-1]

        self.get_logger().info(
            f"Route received ({len(msg.poses)} poses). "
            f"Starting pipeline: "
            f"({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) -> "
            f"({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})"
        )

        # Cancel any active follow goal before starting new pipeline
        if (
            self._follow_goal_handle is not None
            and self._follow_goal_handle.status == GoalStatus.STATUS_EXECUTING
        ):
            self.get_logger().info("Cancelling previous goal...")
            cancel_future = self._follow_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _: self._step1_compute_path(start, goal)
            )
        else:
            self._step1_compute_path(start, goal)

    # ══════════════════════════════════════════════════════════════════════════
    # RECOVERY STATE MACHINE
    # ══════════════════════════════════════════════════════════════════════════

    def _handle_failure(self, stage: str) -> None:
        """Unified failure handler — escalates through recovery levels."""
        if not self._navigating:
            return

        level = self._recovery_level
        self.get_logger().warn(
            f"{stage} failed — recovery level: {level.name}"
        )

        if level == RecoveryLevel.WAIT:
            self._recovery_level = RecoveryLevel.BACKUP
            self.get_logger().info(
                f"[Recovery: WAIT] Waiting {self._replan_delay}s then replanning..."
            )
            if self._replan_timer is not None:
                self._replan_timer.cancel()
            self._replan_timer = self.create_timer(
                self._replan_delay, self._replan_once
            )

        elif level == RecoveryLevel.BACKUP:
            self._recovery_level = RecoveryLevel.CLEAR_COSTMAPS
            self.get_logger().info(
                f"[Recovery: BACKUP] Backing up {self._backup_distance}m..."
            )
            self._execute_backup()

        elif level == RecoveryLevel.CLEAR_COSTMAPS:
            self._recovery_level = RecoveryLevel.ABORT
            self.get_logger().info(
                "[Recovery: CLEAR_COSTMAPS] Clearing global and local costmaps..."
            )
            self._clear_costmaps_and_replan()

        elif level == RecoveryLevel.ABORT:
            self.get_logger().error(
                "[Recovery: ABORT] All recovery attempts exhausted. "
                "Navigation aborted — operator must take over."
            )
            self._recovery_level = RecoveryLevel.WAIT
            self._navigating = False

    # ── BackUp behavior ──────────────────────────────────────────────────────

    def _execute_backup(self) -> None:
        if not self._backup_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "BackUp action server not available — is behavior_server running?"
            )
            # Skip backup, go straight to replan
            self._replan_once()
            return

        goal_msg = BackUp.Goal()
        goal_msg.target.x = -self._backup_distance  # negative = backward
        goal_msg.speed = self._backup_speed
        goal_msg.time_allowance.sec = int(self._backup_time_allowance)

        future = self._backup_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_backup_response)

    def _on_backup_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("BackUp goal rejected, proceeding to replan.")
            self._replan_once()
            return

        self.get_logger().info("BackUp goal accepted, waiting for completion...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_backup_result)

    def _on_backup_result(self, future) -> None:
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("BackUp completed successfully.")
        else:
            self.get_logger().warn(
                f"BackUp finished with status {result.status}, proceeding to replan."
            )
        self._replan_once()

    # ── Clear costmaps ───────────────────────────────────────────────────────

    def _clear_costmaps_and_replan(self) -> None:
        cleared = 0

        def _on_clear_done(future, name):
            nonlocal cleared
            try:
                future.result()
                self.get_logger().info(f"Cleared {name} costmap.")
            except Exception as e:
                self.get_logger().warn(f"Failed to clear {name} costmap: {e}")
            cleared += 1
            if cleared == 2:
                self._replan_once()

        if self._clear_global_costmap_cli.wait_for_service(timeout_sec=2.0):
            f = self._clear_global_costmap_cli.call_async(
                ClearEntireCostmap.Request()
            )
            f.add_done_callback(lambda fut: _on_clear_done(fut, "global"))
        else:
            self.get_logger().warn("Global costmap clear service not available.")
            cleared += 1

        if self._clear_local_costmap_cli.wait_for_service(timeout_sec=2.0):
            f = self._clear_local_costmap_cli.call_async(
                ClearEntireCostmap.Request()
            )
            f.add_done_callback(lambda fut: _on_clear_done(fut, "local"))
        else:
            self.get_logger().warn("Local costmap clear service not available.")
            cleared += 1

        # If both services were unavailable, replan immediately
        if cleared == 2:
            self._replan_once()

    # ══════════════════════════════════════════════════════════════════════════
    # PIPELINE STEP 1: ComputePathToPose (planner_server)
    # ══════════════════════════════════════════════════════════════════════════

    def _step1_compute_path(
        self, start: PoseStamped, goal: PoseStamped
    ) -> None:
        if not self._compute_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "compute_path_to_pose action server not available — "
                "is planner_server running and active?"
            )
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start
        goal_msg.planner_id = self._planner_id
        goal_msg.use_start = True

        self.get_logger().info(
            f"[Step 1/3] Computing path with planner '{self._planner_id}'..."
        )
        future = self._compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_compute_path_response)

    def _on_compute_path_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("ComputePathToPose goal rejected.")
            self._handle_failure("Planner (goal rejected)")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_compute_path_result)

    def _on_compute_path_result(self, future) -> None:
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"ComputePathToPose failed (status={result.status})."
            )
            self._handle_failure("Planner")
            return

        path = result.result.path
        self.get_logger().info(
            f"[Step 1/3] Planner produced path with {len(path.poses)} poses."
        )
        self._step2_smooth_path(path)

    # ══════════════════════════════════════════════════════════════════════════
    # PIPELINE STEP 2: SmoothPath (smoother_server)
    # ══════════════════════════════════════════════════════════════════════════

    def _step2_smooth_path(self, path: Path) -> None:
        if not self._smooth_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "smooth_path action server not available — "
                "is smoother_server running and active?"
            )
            return

        goal_msg = SmoothPath.Goal()
        goal_msg.path = path
        goal_msg.smoother_id = self._smoother_id

        self.get_logger().info(
            f"[Step 2/3] Smoothing path with '{self._smoother_id}'..."
        )
        future = self._smooth_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_smooth_path_response)

    def _on_smooth_path_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("SmoothPath goal rejected.")
            self._handle_failure("Smoother (goal rejected)")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_smooth_path_result)

    def _on_smooth_path_result(self, future) -> None:
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"SmoothPath failed (status={result.status})."
            )
            self._handle_failure("Smoother")
            return

        smoothed_path = result.result.path
        self.get_logger().info(
            f"[Step 2/3] Smoother produced path with "
            f"{len(smoothed_path.poses)} poses."
        )
        self._step3_follow_path(smoothed_path)

    # ══════════════════════════════════════════════════════════════════════════
    # PIPELINE STEP 3: FollowPath (controller_server)
    # ══════════════════════════════════════════════════════════════════════════

    def _step3_follow_path(self, path: Path) -> None:
        if not self._follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "follow_path action server not available — "
                "is controller_server running and active?"
            )
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = "FollowPath"

        self.get_logger().info(
            f"[Step 3/3] Following path ({len(path.poses)} poses)..."
        )
        future = self._follow_path_client.send_goal_async(
            goal_msg, feedback_callback=self._follow_feedback_cb
        )
        future.add_done_callback(self._on_follow_response)

    def _on_follow_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected by controller_server.")
            self._follow_goal_handle = None
            self._handle_failure("Controller (goal rejected)")
            return

        self.get_logger().info("FollowPath goal accepted.")
        self._follow_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_follow_result)

    def _follow_feedback_cb(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Distance to goal: {fb.distance_to_goal:.2f} m  "
            f"speed: {fb.speed:.2f} m/s",
            throttle_duration_sec=2.0,
        )

    def _on_follow_result(self, future) -> None:
        result = future.result()
        status = result.status
        self._follow_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully.")
            self._recovery_level = RecoveryLevel.WAIT
            self._navigating = False
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Goal was cancelled.")
            return

        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                "Goal aborted — path may be blocked by obstacle."
            )
            self._handle_failure("Controller")
            return

        self.get_logger().warn(f"Goal finished with unexpected status: {status}")

    # ── Replan logic ─────────────────────────────────────────────────────────

    def _replan_once(self) -> None:
        """One-shot: request alternate route from route_server, then re-run pipeline."""
        if self._replan_timer is not None:
            self._replan_timer.cancel()
            self._replan_timer = None

        if not self._navigating:
            return

        if self._original_goal is None:
            self.get_logger().error("No original goal cached, cannot replan.")
            return

        if self._amcl_pose is None:
            self.get_logger().error(
                "No AMCL pose received yet, cannot determine start for replan."
            )
            return

        if not self._get_plan_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "route_server/get_plan service not available, cannot replan."
            )
            return

        # Build GetPlan request: current pose -> original goal
        req = GetPlan.Request()
        req.start = PoseStamped()
        req.start.header = self._amcl_pose.header
        req.start.pose = self._amcl_pose.pose.pose
        req.goal = self._original_goal

        self.get_logger().info(
            f"Replanning: ({req.start.pose.position.x:.2f}, "
            f"{req.start.pose.position.y:.2f}) -> "
            f"({req.goal.pose.position.x:.2f}, {req.goal.pose.position.y:.2f})"
        )

        future = self._get_plan_cli.call_async(req)
        future.add_done_callback(self._on_replan_result)

    def _on_replan_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Replan service call failed: {e}")
            self._handle_failure("Replan service")
            return

        path = response.plan
        if len(path.poses) == 0:
            self.get_logger().warn("Route server returned empty path on replan.")
            self._handle_failure("Replan (empty path)")
            return

        self.get_logger().info(
            f"Replan succeeded — new route with {len(path.poses)} poses. "
            f"Re-running pipeline."
        )

        # Feed the new route through the full pipeline
        start = path.poses[0]
        goal = path.poses[-1]
        self._step1_compute_path(start, goal)


def main() -> None:
    rclpy.init()
    node = LinearOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
