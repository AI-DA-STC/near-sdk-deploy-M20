"""
navigation.launch.py — Consolidated Nav2 navigation stack for M20 quadruped.

Launches the full navigation pipeline:

  Localization:
    1. pointcloud_to_laserscan  — 3D PointCloud2 -> 2D LaserScan
    2. rf2o_laser_odometry      — LaserScan -> Odometry for EKF
    3. ekf_node                 — Fuses LiDAR odom + IMU -> odom->base_link TF
    4. map_server               — Serves 2D occupancy grid
    5. amcl                     — Publishes map->odom TF
    6. static TFs               — Gazebo sensor frame aliases

  Navigation:
    7.  route_server            — SWAGGER graph -> sparse waypoint route
    8.  planner_server          — SmacPlanner2D on global costmap -> dense path
    9.  smoother_server         — Savitzky-Golay path smoothing
    10. controller_server       — DWB local planner -> cmd_vel
    11. linear_orchestrator     — Orchestrates planner->smoother->controller
    12. m20_cmd_vel_bridge      — /cmd_vel -> /M20/cmd_vel with clamping

  Lifecycle:
    13. lifecycle_manager_navigation — Manages map_server, amcl, planner_server,
                                       smoother_server, controller_server

Prerequisite launches:
  - gazebo_velodyne.launch.py  (Gazebo simulation + sensor bridges)
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pyprojroot import here

PROJECT_ROOT = here()
PACKAGE_PATH = PROJECT_ROOT / "src" / "M20_sdk_deploy"

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
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    declare_args = [
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
            default_value="true",
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
        # Python nodes (route_server, orchestrator, bridge) flush output
        # immediately so nothing is lost from launch.log on crash
        SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
    ]

    # ══════════════════════════════════════════════════════════════════════
    # LOCALIZATION NODES
    # ══════════════════════════════════════════════════════════════════════

    # Static TFs for Gazebo frame names
    static_tf_gazebo_velodyne = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_velodyne_frame_tf",
        arguments=["0.3", "0", "0.145", "0", "0", "0",
                    "base_link", "M20/base_link/velodyne_hdl32e"],
    )

    static_tf_gazebo_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_imu_frame_tf",
        arguments=["0.0632", "-0.0268", "-0.0435", "0", "0", "0",
                    "base_link", "M20/base_link/imu_sensor"],
    )

    # Convert 3D PointCloud2 -> 2D LaserScan for AMCL
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[{
            "use_sim_time": True,
            "target_frame": "base_link",
            "min_height": -0.1,
            "max_height": 0.5,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            # 2*pi/1024 — must match the gpu_lidar horizontal sample count in
            # M20_velodyne.sdf, else the scan is padded with interleaved inf bins.
            "angle_increment": 0.00614,
            "range_min": 0.9,
            "range_max": 70.0,
            "inf_epsilon": 1.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/M20/LIDAR/VELODYNE"),
            ("scan", "/M20/scan"),
        ],
    )

    # rf2o scan-matching odometry
    rf2o_laser_odometry = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[{
            "use_sim_time": True,
            "laser_scan_topic": "/M20/scan",
            "odom_topic": "/odom_rf2o",
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "init_pose_from_topic": "",
            "publish_tf": False,
            "freq": 10.0,
        }],
    )

    # EKF — fuses LiDAR odom + IMU to publish odom -> base_link
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[params_file],
    )

    # Map server — serves the 2D occupancy grid
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[
            params_file,
            {"yaml_filename": map_yaml},
        ],
    )

    # AMCL — publishes map -> odom transform
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[params_file],
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

    # Controller server — DWB local planner
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # Behavior server — recovery behaviors (BackUp) — DISABLED
    # (orchestrator skips BACKUP recovery when the action server is absent)
    # behavior_server = Node(
    #     package="nav2_behaviors",
    #     executable="behavior_server",
    #     name="behavior_server",
    #     output="screen",
    #     parameters=[params_file, {"use_sim_time": use_sim_time}],
    # )

    # Linear orchestrator — drives planner->smoother->controller pipeline
    linear_orchestrator = Node(
        package="rl_deploy",
        executable="linear_orchestrator.py",
        name="linear_orchestrator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Velocity bridge — /cmd_vel -> /M20/cmd_vel with clamping
    cmd_vel_bridge = Node(
        package="rl_deploy",
        executable="m20_cmd_vel_bridge.py",
        name="m20_cmd_vel_bridge",
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
        parameters=[params_file],
    )

    # ══════════════════════════════════════════════════════════════════════

    return LaunchDescription(
        declare_args
        + log_env
        + [
            # Localization (order matters for TF readiness)
            static_tf_gazebo_velodyne,
            static_tf_gazebo_imu,
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
            # behavior_server,  # disabled — BackUp recovery not used
            linear_orchestrator,
            cmd_vel_bridge,
            # Lifecycle (activates map_server, amcl, planner, smoother, controller)
            lifecycle_manager,
        ]
    )
