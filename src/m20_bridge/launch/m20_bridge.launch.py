"""
m20_bridge.launch.py — DeepRobotics Lynx M20 Pro bridge for the robot-agnostic
nav_core (rl_deploy navigation.launch.py sim:=false).

Runs on the GOS host (10.21.31.104), typically in its own container with
--network=host --ipc=host. Supplies nav_core's generic contract and converts
its /cmd_vel back to the robot:

  m20_udp_node    UDP Inspection Protocol <-> /joint_states, /m20/telemetry,
                  /robot/ready, and (when enable_tx) /cmd_vel -> axis commands
  restamp_imu     /IMU          -> /imu   (offset: preserve 200Hz dt)
  restamp_lidar   /LIDAR/POINTS -> /lidar/points (offset)
  rtsp_camera1/2  RTSP video1/2 (AOS :8554) -> /camera1, /camera2
  static TF       base_link -> lidar_link
  robot_state_publisher   M20 URDF (from rl_deploy) -> /robot_description + TF

/IMU and /LIDAR/POINTS reach this host directly from the M20, so no AOS-side
relay is involved. They still carry the robot's clock, so both use 'offset' to
rebase onto the GOS clock while PRESERVING inter-sample dt — 'arrival' would
collapse it, which hurts the LIO. /LIO_ODOM is dropped: the GOS-side LIO
produces odometry locally.

restamp reliability=best_effort matches the M20 publisher (a reliable pub
satisfies a best_effort sub); a mismatch would silently match nothing, so
restamp_relay logs a loud INCOMPATIBLE-QoS error if it sees one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    enable_cameras = LaunchConfiguration("enable_cameras")

    args = [
        DeclareLaunchArgument(
            "enable_tx", default_value="false",
            description="stream /cmd_vel as UDP axis commands (2/21). "
                        "Keep false until the Cmd-21 path is validated (T4-T6)."),
        DeclareLaunchArgument("robot_ip", default_value="10.21.31.103"),
        DeclareLaunchArgument(
            "viz_lidar2", default_value="false",
            description="also relay the 2nd lidar cloud for RViz (extra CPU)"),
        DeclareLaunchArgument(
            "enable_cameras", default_value="true",
            description="pull the two RTSP camera streams -> /camera1, /camera2 "
                        "(set false to save bandwidth/CPU)"),
    ]

    udp_node = Node(
        package="m20_bridge", executable="m20_udp_node.py",
        name="m20_udp_node", output="screen",
        parameters=[params, {"enable_tx": enable_tx, "robot_ip": robot_ip}],
    )

    restamp_imu = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_imu",
        output="screen", parameters=[{
            "msg_type": "imu", "input_topic": "/IMU",
            "output_topic": "/imu", "frame_id": "imu_link",
            "mode": "offset", "reliability": "best_effort"}])

    # /LIDAR/POINTS carries the robot's clock (not this host's) — use 'offset' to
    # rebase onto the GOS clock while preserving the 10 Hz spacing the LIO relies on.
    restamp_lidar = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_lidar",
        output="screen", parameters=[{
            "msg_type": "pointcloud2", "input_topic": "/LIDAR/POINTS",
            "output_topic": "/lidar/points", "frame_id": "lidar_link",
            "mode": "offset", "reliability": "best_effort"}])

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

    # M20 wide-angle cameras over RTSP (vendor manual Appendix 3), decoded and
    # republished as sensor_msgs/Image. RTSP is plain TCP, so unlike the DDS
    # sensors these are reachable straight from the GOS.
    def _camera(name, stream, topic, frame):
        return Node(
            package="m20_bridge", executable="rtsp_camera_node.py",
            name=name, output="screen", condition=IfCondition(enable_cameras),
            parameters=[{
                "rtsp_url": ParameterValue(
                    ["rtsp://", robot_ip, ":8554/", stream], value_type=str),
                "output_topic": topic, "frame_id": frame, "fps": 15.0}])

    cam1 = _camera("rtsp_camera1", "video1", "/camera1", "camera1_link")
    cam2 = _camera("rtsp_camera2", "video2", "/camera2", "camera2_link")

    return LaunchDescription(args + [
        udp_node,
        restamp_imu, restamp_lidar, restamp_lidar2,
        cam1, cam2,
        static_tf_lidar, rsp,
    ])
