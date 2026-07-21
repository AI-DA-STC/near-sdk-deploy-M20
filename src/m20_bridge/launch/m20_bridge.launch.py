"""
m20_bridge.launch.py — DeepRobotics Lynx M20 Pro bridge for the robot-agnostic
nav_core (rl_deploy navigation.launch.py sim:=false).

Runs on the GOS host (10.21.31.104), typically in its own container with
--network=host --ipc=host. Supplies nav_core's generic contract and converts
its /cmd_vel back to the robot:

  m20_udp_node    UDP Inspection Protocol <-> /joint_states, /m20/telemetry,
                  /robot/ready, and (when enable_tx) /cmd_vel -> axis commands
  restamp_imu     /IMU_YESENSE (AOS DDS) -> /imu     (offset: preserve 200Hz dt)
  restamp_odom    /LIO_ODOM   (AOS DDS)  -> /odom     (offset)
  restamp_lidar   /LIDAR/POINTS (GOS rsdriver) -> /lidar/points (arrival = rename)
  static TF       base_link -> lidar_link
  robot_state_publisher   M20 URDF (from rl_deploy) -> /robot_description + TF

Reliability of the AOS DDS publishers is confirmed in test T1; the values below
are the common defaults (sensor topics best_effort, odom reliable). A wrong
value makes the subscription silently match nothing — restamp_relay logs a loud
INCOMPATIBLE-QoS error if so.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description():
    """M20 URDF with absolute mesh URIs (asset shipped in rl_deploy)."""
    rl = get_package_share_directory("rl_deploy")
    urdf = os.path.join(rl, "model", "M20_urdf", "urdf", "M20.urdf")
    meshes = os.path.join(rl, "model", "M20_urdf", "meshes")
    with open(urdf) as f:
        return f.read().replace('filename="../meshes/',
                                f'filename="file://{meshes}/')


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("m20_bridge"), "config", "bridge_params.yaml")

    enable_tx = LaunchConfiguration("enable_tx")
    robot_ip = LaunchConfiguration("robot_ip")
    viz_lidar2 = LaunchConfiguration("viz_lidar2")

    args = [
        DeclareLaunchArgument(
            "enable_tx", default_value="false",
            description="stream /cmd_vel as UDP axis commands (2/21). "
                        "Keep false until the Cmd-21 path is validated (T4-T6)."),
        DeclareLaunchArgument("robot_ip", default_value="10.21.31.103"),
        DeclareLaunchArgument(
            "viz_lidar2", default_value="false",
            description="also relay the 2nd lidar cloud for RViz (extra CPU)"),
    ]

    udp_node = Node(
        package="m20_bridge", executable="m20_udp_node.py",
        name="m20_udp_node", output="screen",
        parameters=[params, {"enable_tx": enable_tx, "robot_ip": robot_ip}],
    )

    restamp_imu = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_imu",
        output="screen", parameters=[{
            "msg_type": "imu", "input_topic": "/IMU_YESENSE",
            "output_topic": "/imu", "frame_id": "imu_link",
            "mode": "offset", "reliability": "best_effort"}])

    restamp_odom = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_odom",
        output="screen", parameters=[{
            "msg_type": "odometry", "input_topic": "/LIO_ODOM",
            "output_topic": "/odom", "frame_id": "odom",
            "child_frame_id": "base_link", "mode": "offset",
            "reliability": "reliable"}])

    # rsdriver runs on THIS host, so /LIDAR/POINTS already carries the GOS clock;
    # 'arrival' mode is effectively a cheap topic+frame rename.
    restamp_lidar = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_lidar",
        output="screen", parameters=[{
            "msg_type": "pointcloud2", "input_topic": "/LIDAR/POINTS",
            "output_topic": "/lidar/points", "frame_id": "lidar_link",
            "mode": "arrival", "reliability": "best_effort"}])

    restamp_lidar2 = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_lidar2",
        output="screen", condition=IfCondition(viz_lidar2), parameters=[{
            "msg_type": "pointcloud2", "input_topic": "/LIDAR/POINTS2",
            "output_topic": "/lidar/points2", "frame_id": "lidar_link",
            "mode": "arrival", "reliability": "best_effort"}])

    # Front lidar body offset per manual §1.10 (320.28, 0, -13) mm.
    # TODO(calibration T3): verify origin/tilt via a ground-plane check in RViz.
    static_tf_lidar = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="lidar_link_static_tf",
        arguments=["0.32028", "0", "-0.013", "0", "0", "0",
                   "base_link", "lidar_link"])

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="m20_robot_state_publisher", output="screen",
        parameters=[{"robot_description": _robot_description(),
                     "use_sim_time": False}])

    return LaunchDescription(args + [
        udp_node,
        restamp_imu, restamp_odom, restamp_lidar, restamp_lidar2,
        static_tf_lidar, rsp,
    ])
