"""
navigation.launch.py — ROBOT-AGNOSTIC Nav2 + routing stack (nav_core).

This is the "black box": it consumes GENERIC topics and produces /cmd_vel. It
contains NO robot-specific transport, clock, frame, or URDF knowledge. A
per-robot *bridge* (e.g. m20_bridge.launch.py, or in simulation the gazebo
bringup) is responsible for supplying the generic inputs and consuming the
output. Porting to a new robot = write a new bridge launch file; this file is
untouched.

Generic contract:
  IN   /lidar/points   sensor_msgs/PointCloud2   (front lidar)
  IN   /imu            sensor_msgs/Imu           (raw; EKF yaw-rate, GLIM)
  IN   /odom           nav_msgs/Odometry         (robot's fused odometry)
  IN   /joint_states   + /robot_description + TF base_link->lidar_link
                                                 (published by the bridge)
  IN   /goal_pose, /initialpose                  (RViz on the workstation)
  OUT  /cmd_vel        geometry_msgs/Twist       (raw DWB output; the bridge
                                                  clamps + converts to the robot)

Internal pipeline:
  /lidar/points -> pointcloud_to_laserscan -> /scan -> rf2o -> /odom
                                                    -> amcl + costmaps
  /odom (+/imu) -> ekf -> odom->base_link TF, /odometry/filtered
  /goal_pose    -> route_server (graph) -> linear_orchestrator -> nav2 -> /cmd_vel

ODOMETRY IS rf2o IN BOTH MODES. The robot publishes its own onboard LIO fusion
on /LIO_ODOM, but the stack deliberately does not consume it: /odom is always
scan-matched from /scan by rf2o, so sim and robot run the identical localization
pipeline and the seam stays at the sensor topics. rf2o and ekf_node therefore
carry no `sim` condition.

Modes (sim launch argument): both modes consume the SAME generic topics; `sim`
only toggles the helper nodes that exist because Gazebo has a clean clock and
model-scoped sensor frame names instead of a hardware bridge.
  sim:=true  (default) — pair with the Gazebo bringup (gazebo_velodyne.launch.py),
                         which publishes /lidar/points, /imu, /joint_states.
                         Adds the Gazebo sensor-frame TFs.
  sim:=false            — pair with a robot bridge (m20_bridge.launch.py) that
                         publishes /lidar/points, /imu, /joint_states and the
                         base_link->lidar_link TF. It does NOT supply /odom.
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pyprojroot import here

PROJECT_ROOT = here()

# ── Run logs: navigation_logs/<timestamp>/ at the project root ──────────
# Per-node rcutils .log files land here directly (via ROS_LOG_DIR below).
# launch.log is already bound to ~/.ros/log/<run>/ before this file is
# imported, so it is exposed here as a symlink instead of being moved.
LOG_DIR = PROJECT_ROOT / "navigation_logs" / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_DIR.mkdir(parents=True, exist_ok=True)

_launch_log = os.path.join(launch_config.log_dir, "launch.log")
try:
    (LOG_DIR / "launch.log").symlink_to(_launch_log)
except FileExistsError:
    pass

# Convenience symlink: navigation_logs/latest -> newest run
_latest = LOG_DIR.parent / "latest"
if _latest.is_symlink() or _latest.exists():
    _latest.unlink()
_latest.symlink_to(LOG_DIR, target_is_directory=True)


def generate_launch_description():
    pkg_share = get_package_share_directory("rl_deploy")
    default_params = os.path.join(
        pkg_share, "config", "nav2", "navigation_params.yaml"
    )
    default_map = str(PROJECT_ROOT / "maps" / "edifice_SLAM_v0_2d.yaml")
    default_graph = str(PROJECT_ROOT / "maps" / "graph.gml")

    # ── Launch arguments ──────────────────────────────────────────────────

    map_yaml = LaunchConfiguration("map")
    graph_file = LaunchConfiguration("graph_file")
    sim = LaunchConfiguration("sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    declare_args = [
        DeclareLaunchArgument(
            "sim",
            default_value="true",
            description="true = pair with Gazebo bringup; "
                        "false = pair with a robot bridge (m20_bridge.launch.py)",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Path to 2D map .yaml file",
        ),
        DeclareLaunchArgument(
            "graph_file",
            default_value=default_graph,
            description="Absolute path to the SWAGGER GML waypoint graph",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            # Follows `sim` unless explicitly overridden.
            default_value=sim,
            description="Use simulation (Gazebo) clock",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to consolidated navigation params YAML",
        ),
    ]

    # ── Logging environment for all launched nodes ────────────────────────
    log_env = [
        # rcutils writes each node's own .log file into this run's directory
        SetEnvironmentVariable("ROS_LOG_DIR", str(LOG_DIR)),
        # Python nodes (route_server, orchestrator) flush output immediately
        # so nothing is lost from launch.log on crash
        SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
    ]

    # ══════════════════════════════════════════════════════════════════════
    # LOCALIZATION
    # ══════════════════════════════════════════════════════════════════════

    # ── Sim-only: static TFs mapping Gazebo sensor frames to base_link ────
    # (On the robot the bridge publishes base_link->lidar_link and the URDF
    #  supplies the rest; in Gazebo the sensor frames carry model-scoped names.)
    static_tf_gazebo_velodyne = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_velodyne_frame_tf",
        arguments=["0.3", "0", "0.145", "0", "0", "0",
                    "base_link", "M20/base_link/velodyne_hdl32e"],
        condition=IfCondition(sim),
    )

    static_tf_gazebo_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_imu_frame_tf",
        arguments=["0.0632", "-0.0268", "-0.0435", "0", "0", "0",
                    "base_link", "M20/base_link/imu_sensor"],
        condition=IfCondition(sim),
    )

    # Convert 3D PointCloud2 -> 2D LaserScan for AMCL + costmaps.
    # Generic input /lidar/points in BOTH modes (bridge or Gazebo supplies it).
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "base_link",
            "min_height": -0.1,
            "max_height": 0.5,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            # 2*pi/1024 — matches the sim gpu_lidar horizontal sample count in
            # M20_velodyne.sdf; also a sane bin width for the real merged cloud.
            "angle_increment": 0.00614,
            "range_min": 0.9,
            "range_max": 70.0,
            "inf_epsilon": 1.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/lidar/points"),
            ("scan", "/scan"),
        ],
    )

    # rf2o scan-matching odometry -> /odom. Runs in BOTH modes: /odom has no
    # other source, and the robot's own /LIO_ODOM is deliberately not used.
    # rf2o consumes /scan, which pointcloud_to_laserscan produces identically in
    # sim and on the robot, so this node is genuinely mode-agnostic.
    #
    # publish_tf: False — odom->base_link belongs to ekf_node alone. A second
    # publisher of that edge would fight it.
    rf2o_laser_odometry = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[{
            "use_sim_time": use_sim_time,
            "laser_scan_topic": "/scan",
            "odom_topic": "/odom",
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "init_pose_from_topic": "",
            "publish_tf": False,
            "freq": 10.0,
        }],
    )

    # EKF — publishes odom -> base_link and /odometry/filtered.
    # Fuses rf2o (x, y) + /imu (yaw rate), per navigation_params.yaml. Identical
    # in both modes now that rf2o is the /odom source in both: there is no
    # longer a robot-specific variant that trusted an onboard LIO's absolute yaw.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # Map server — serves the 2D occupancy grid
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[
            params_file,
            # params_file hardcodes use_sim_time: true — override wins here
            {"yaml_filename": map_yaml, "use_sim_time": use_sim_time},
        ],
    )

    # AMCL — publishes map -> odom transform
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ══════════════════════════════════════════════════════════════════════
    # NAVIGATION NODES
    # ══════════════════════════════════════════════════════════════════════

    # Route server — SWAGGER graph planner
    route_server = Node(
        package="rl_deploy",
        executable="route_server.py",
        name="route_server",
        output="screen",
        parameters=[
            params_file,
            {
                "graph_file": graph_file,
                "map_yaml": map_yaml,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # Planner server — SmacPlanner2D on global costmap
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # Smoother server — Savitzky-Golay path smoothing
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # Controller server — DWB local planner (publishes /cmd_vel)
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # Linear orchestrator — drives planner->smoother->controller pipeline
    linear_orchestrator = Node(
        package="rl_deploy",
        executable="linear_orchestrator.py",
        name="linear_orchestrator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ══════════════════════════════════════════════════════════════════════
    # LIFECYCLE MANAGER
    # ══════════════════════════════════════════════════════════════════════

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ══════════════════════════════════════════════════════════════════════

    return LaunchDescription(
        declare_args
        + log_env
        + [
            # Localization (order matters for TF readiness)
            static_tf_gazebo_velodyne,      # sim only
            static_tf_gazebo_imu,           # sim only
            pointcloud_to_laserscan,
            rf2o_laser_odometry,
            ekf_node,
            map_server,
            amcl,
            # Navigation
            route_server,
            planner_server,
            smoother_server,
            controller_server,
            linear_orchestrator,
            # Lifecycle (activates map_server, amcl, planner, smoother, controller)
            lifecycle_manager,
        ]
    )
