"""
m20_bridge.launch.py — DeepRobotics Lynx M20 Pro bridge for the robot-agnostic
nav_core (rl_deploy navigation.launch.py sim:=false).

Runs on the GOS host (10.21.31.104), typically in its own container with
--network=host --ipc=host. Supplies nav_core's generic contract and converts
its /cmd_vel back to the robot:

  m20_udp_node    UDP Inspection Protocol <-> /m20/telemetry, /robot/ready,
                  and (when enable_tx) /cmd_vel -> axis commands
  m20_joints_node /JOINTS_DATA (drdds) -> /joint_states  [Fast-DDS process]
  joints_uds_*    that JointState -> unix socket -> CycloneDDS
  imu_uds_source  /IMU -> unix socket   [Fast-DDS process — see below]
  imu_uds_sink    unix socket -> /IMU_fastdds
  restamp_imu     /IMU_fastdds  -> /imu   (offset: preserve 200Hz dt)
  odom_uds_*      /ODOM -> unix socket -> /ODOM_fastdds  [Fast-DDS process]
  restamp_odom    /ODOM_fastdds -> /odom (offset: preserve sample spacing)
  restamp_lidar   /LIDAR/POINTS -> /lidar/points (offset)
  rtsp_camera1/2  RTSP video1/2 (AOS :8554) -> /camera1, /camera2
  static TF       base_link -> lidar_link
  robot_state_publisher   M20 URDF (from rl_deploy) -> /robot_description + TF

THE TWO SENSORS NEED DIFFERENT MIDDLEWARES. THIS IS THE WHOLE COMPLICATION.
The M20 publishes /LIDAR/POINTS (~1.2 MB, 10 Hz) and /IMU (~300 B, 200 Hz) from
Foxy over Fast-DDS, which is a manufacturer setting. Measured on the robot from
this Humble container:

    topic            CycloneDDS 0.10        Fast-DDS 2.6
    /LIDAR/POINTS    9.3 Hz, 10.8 MB/s      matched NOTHING
    /IMU             matched NOTHING        200.0 Hz

Exactly complementary, and RMW_IMPLEMENTATION is per-PROCESS, so no single
process can read both. "Matched nothing" is a discovery-level failure, not lossy
delivery — no amount of QoS or buffer tuning moves it.

The resolution is two processes with different RMWs, joined by a channel that is
not DDS at all:

    imu_uds_source  [Fast-DDS]   /IMU  --raw CDR-->  unix socket
    imu_uds_sink    [CycloneDDS] socket --raw CDR-->  /IMU_fastdds
    restamp_imu     [CycloneDDS] /IMU_fastdds -> /imu

Everything else — restamp_lidar, m20_udp_node, the cameras, TF, RSP — runs on
the container's primary RMW, CycloneDDS. /IMU is the stream that gets hopped
rather than the cloud because it is ~60 kB/s against the cloud's ~12 MB/s: the
lidar stays native and never enters the relay. See scripts/cdr_uds_relay.py, and
scripts/test_humble_ingest.sh to re-measure the table above at any time.

Set imu_via_fastdds:=false to drop the hop and subscribe /IMU directly on the
container's own RMW — correct only if that RMW can actually see /IMU.

/JOINTS_DATA takes the same treatment (joints_via_fastdds), because it comes
from the same Foxy control stack as /IMU. It replaces the UDP MotorStatus path
for joint states: same 16 motors, but at the control rate and with velocity and
torque, and m20_joints_node converts the vendor's raw motor angles into the
URDF's joint convention on the way through. Without that conversion the URDF
renders inverted — see the node's docstring for the calibration and its
provenance.

/ODOM gets the same treatment (odom_via_fastdds), because it is the onboard
state estimator's output (joint encoders + foot contact + IMU fusion) and
comes from that same Foxy control stack, not the lidar side. It is nav_core's
/odom: EKF fuses it (with absolute yaw trusted — see navigation.launch.py's
ekf_node_robot) instead of scan-matching one from /scan with rf2o, which was
the previous approach and had no absolute heading reference to correct
dead-reckoned yaw drift. THIS TOPIC WAS RENAMED BY THE VENDOR: it shipped as
/LIO_ODOM and is /ODOM as of the firmware on this robot as of 2026-08-11 (see
the OTA-rename risk this file's DEPLOYMENT.md entry already calls out) — re-run
`ros2 topic info -v /ODOM` after any OTA and update odom_topic below if it
moves again. Also note the vendor stamps header.frame_id="map" and leaves
child_frame_id empty on this message; restamp_odom overrides both to this
stack's odom/base_link naming — do not drop that override.

(Do NOT try to solve this with a CycloneDDS relay in a Foxy container: Cyclone
0.7, which is what Foxy ships, SEGFAULTS in rmw_create_node as soon as it
exchanges discovery with this robot's Fast-DDS participants. Configs that avoid
those participants survive but then receive nothing, because those participants
are the publishers. That approach was built, measured, and removed.)

Stamps arriving here carry the ROBOT's clock, so this file remains the single
place clock differences are corrected. All three hopped restamps use 'offset'
to rebase onto the GOS clock while PRESERVING inter-sample dt; 'arrival' would
collapse it, which hurts the LIO (and would turn /odom's velocity fields to
noise).

restamp reliability matches the M20's publishers, which are RELIABLE. A mismatch
would silently match nothing, so restamp_relay logs a loud INCOMPATIBLE-QoS
error if it sees one.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    imu_topic = LaunchConfiguration("imu_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    lidar2_topic = LaunchConfiguration("lidar2_topic")
    sensor_reliability = LaunchConfiguration("sensor_reliability")
    imu_via_fastdds = LaunchConfiguration("imu_via_fastdds")
    imu_bridge_topic = LaunchConfiguration("imu_bridge_topic")
    imu_socket = LaunchConfiguration("imu_socket")
    fastdds_profile = LaunchConfiguration("fastdds_profile")
    joints_topic = LaunchConfiguration("joints_topic")
    joints_via_fastdds = LaunchConfiguration("joints_via_fastdds")
    joints_bridge_topic = LaunchConfiguration("joints_bridge_topic")
    joints_socket = LaunchConfiguration("joints_socket")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_via_fastdds = LaunchConfiguration("odom_via_fastdds")
    odom_bridge_topic = LaunchConfiguration("odom_bridge_topic")
    odom_socket = LaunchConfiguration("odom_socket")

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
        # Sensor sources: the M20's OWN topics, subscribed directly. This works
        # only because the container runs rmw_cyclonedds_cpp — see the module
        # docstring for the measurement.
        DeclareLaunchArgument(
            "imu_topic", default_value="/IMU",
            description="IMU source — the M20's own publisher"),
        DeclareLaunchArgument(
            "lidar_topic", default_value="/LIDAR/POINTS",
            description="front lidar source — the M20's own publisher"),
        DeclareLaunchArgument(
            "lidar2_topic", default_value="/LIDAR/POINTS2",
            description="2nd lidar source (only used when viz_lidar2:=true)"),
        DeclareLaunchArgument(
            "sensor_reliability", default_value="reliable",
            description="Matches the M20's publishers, which are RELIABLE, so a "
                        "lost fragment of a ~1.2 MB scan is retransmitted rather "
                        "than costing the whole sample. best_effort measured "
                        "equivalent here (9.33 vs 9.25 Hz) and also matches, so "
                        "either works; reliable is kept for the repair."),
        # --- the /IMU cross-RMW hop (see module docstring) -------------------
        DeclareLaunchArgument(
            "imu_via_fastdds", default_value="true",
            description="Read /IMU in a Fast-DDS sidecar process and hop it to "
                        "CycloneDDS over a Unix socket. REQUIRED on this robot: "
                        "CycloneDDS matches nothing on /IMU while Fast-DDS "
                        "matches nothing on /LIDAR/POINTS. Set false to have "
                        "restamp_imu subscribe /IMU directly on the container's "
                        "own RMW — only correct if that RMW can actually see it."),
        DeclareLaunchArgument(
            "imu_bridge_topic", default_value="/IMU_fastdds",
            description="internal CycloneDDS topic the hop's sink republishes "
                        "onto; restamp_imu consumes this when imu_via_fastdds"),
        DeclareLaunchArgument(
            "imu_socket", default_value="/tmp/m20_imu_cdr.sock",
            description="Unix datagram socket joining the two RMW processes. "
                        "Both live in THIS container, so it needs no volume."),
        DeclareLaunchArgument(
            "fastdds_profile",
            default_value="/root/ros_ws/config/fastdds_profile_gos.xml",
            description="UDP-only Fast-DDS profile for the /IMU sidecar — the "
                        "configuration measured to deliver /IMU at 200 Hz"),
        # --- joint states from the control stack's native topic --------------
        DeclareLaunchArgument(
            "joints_topic", default_value="/JOINTS_DATA",
            description="drdds/JointsData source — the M20's own publisher. "
                        "Carries position, velocity AND torque for all 16 "
                        "motors at the control rate; the UDP 1002/4 MotorStatus "
                        "path it replaces had position only."),
        DeclareLaunchArgument(
            "joints_via_fastdds", default_value="true",
            description="Read /JOINTS_DATA in a Fast-DDS sidecar and hop it to "
                        "CycloneDDS over a Unix socket, exactly as /IMU. "
                        "Default true because /JOINTS_DATA comes from the same "
                        "Foxy control stack as /IMU; /LIDAR/POINTS is the "
                        "outlier. Verify with scripts/test_humble_ingest.sh and "
                        "set false if CycloneDDS can see it directly."),
        DeclareLaunchArgument(
            "joints_bridge_topic", default_value="/joint_states_fastdds",
            description="internal Fast-DDS topic m20_joints_node publishes onto "
                        "when hopping; the relay carries it to /joint_states"),
        DeclareLaunchArgument(
            "joints_socket", default_value="/tmp/m20_joints_cdr.sock",
            description="Unix datagram socket joining the two RMW processes"),
        # --- onboard state estimator (joint encoders + foot contact + IMU) ---
        DeclareLaunchArgument(
            "odom_topic", default_value="/ODOM",
            description="Onboard fused-odometry source — the M20's own "
                        "publisher (nav_msgs/Odometry). Shipped as /LIO_ODOM "
                        "on earlier firmware and renamed to /ODOM by an OTA "
                        "update (confirmed via `ros2 topic info -v /ODOM` on "
                        "2026-08-11); re-check after any future OTA."),
        DeclareLaunchArgument(
            "odom_via_fastdds", default_value="true",
            description="Read /ODOM in a Fast-DDS sidecar and hop it to "
                        "CycloneDDS over a Unix socket, exactly as /IMU. "
                        "Default true because /ODOM comes from the same Foxy "
                        "control stack as /IMU and /JOINTS_DATA; /LIDAR/POINTS "
                        "is the outlier. Verify with scripts/test_humble_ingest.sh "
                        "and set false if CycloneDDS can see it directly."),
        DeclareLaunchArgument(
            "odom_bridge_topic", default_value="/ODOM_fastdds",
            description="internal CycloneDDS topic the hop's sink republishes "
                        "onto; restamp_odom consumes this when odom_via_fastdds"),
        DeclareLaunchArgument(
            "odom_socket", default_value="/tmp/m20_odom_cdr.sock",
            description="Unix datagram socket joining the two RMW processes"),
    ]

    # publish_joint_states:=false — m20_joints_node owns /joint_states now. The
    # UDP node keeps telemetry, readiness and the Cmd-21 actuation path.
    udp_node = Node(
        package="m20_bridge", executable="m20_udp_node.py",
        name="m20_udp_node", output="screen",
        parameters=[params, {"enable_tx": enable_tx, "robot_ip": robot_ip,
                             "publish_joint_states": False}],
    )

    # /JOINTS_DATA CROSS-RMW HOP — same shape as the /IMU hop below, and for the
    # same reason: the control stack publishes over Fast-DDS while this
    # container runs CycloneDDS for the lidar. The difference is that the
    # conversion happens BEFORE the socket, so what crosses is an ordinary
    # sensor_msgs/JointState the generic relay already knows how to carry.
    #
    #   m20_joints_node [Fast-DDS] /JOINTS_DATA -> /joint_states_fastdds
    #   joints_uds_source [Fast-DDS]            -> unix socket
    #   joints_uds_sink [CycloneDDS]  socket    -> /joint_states
    _fastdds_env = {
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile,
        "CYCLONEDDS_URI": "",
    }

    joints_node = Node(
        package="m20_bridge", executable="m20_joints_node.py",
        name="m20_joints_node", output="screen",
        condition=IfCondition(joints_via_fastdds),
        additional_env=_fastdds_env,
        parameters=[{"joints_topic": joints_topic,
                     "output_topic": joints_bridge_topic,
                     "reliability": sensor_reliability}])

    joints_source = Node(
        package="m20_bridge", executable="cdr_uds_relay.py",
        name="joints_uds_source", output="screen",
        condition=IfCondition(joints_via_fastdds),
        additional_env=_fastdds_env,
        parameters=[{
            "role": "source", "msg_type": "jointstate",
            "topic": joints_bridge_topic, "socket_path": joints_socket,
            "reliability": sensor_reliability,
            "depth": 20, "stats_period": 10.0}])

    joints_sink = Node(
        package="m20_bridge", executable="cdr_uds_relay.py",
        name="joints_uds_sink", output="screen",
        condition=IfCondition(joints_via_fastdds),
        parameters=[{
            "role": "sink", "msg_type": "jointstate",
            "topic": "/joint_states", "socket_path": joints_socket,
            "reliability": "reliable",
            "depth": 20, "stats_period": 10.0}])

    # No hop: /JOINTS_DATA is readable on the container's own RMW.
    joints_node_direct = Node(
        package="m20_bridge", executable="m20_joints_node.py",
        name="m20_joints_node", output="screen",
        condition=UnlessCondition(joints_via_fastdds),
        parameters=[{"joints_topic": joints_topic,
                     "output_topic": "/joint_states",
                     "reliability": sensor_reliability}])

    # /IMU CROSS-RMW HOP. Two processes, different RMWs, joined by a Unix socket
    # because RMW_IMPLEMENTATION is per-process and each middleware can only see
    # one of the M20's two sensor streams. launch's additional_env is what makes
    # the split possible inside one container: each Node is its own process, so
    # each can have its own RMW.
    #
    # /IMU is the stream that gets hopped (not the cloud) because it is ~60 kB/s
    # against the cloud's ~12 MB/s — the lidar stays native on CycloneDDS.
    imu_source = Node(
        package="m20_bridge", executable="cdr_uds_relay.py",
        name="imu_uds_source", output="screen",
        condition=IfCondition(imu_via_fastdds),
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "FASTRTPS_DEFAULT_PROFILES_FILE": fastdds_profile,
            # Belt and braces: stop Fast-DDS from ever reading a CycloneDDS
            # config that happens to be in the container's environment.
            "CYCLONEDDS_URI": "",
        },
        parameters=[{
            "role": "source", "msg_type": "imu", "topic": imu_topic,
            "socket_path": imu_socket, "reliability": sensor_reliability,
            "depth": 20, "stats_period": 10.0}])

    imu_sink = Node(
        package="m20_bridge", executable="cdr_uds_relay.py",
        name="imu_uds_sink", output="screen",
        condition=IfCondition(imu_via_fastdds),
        parameters=[{
            "role": "sink", "msg_type": "imu", "topic": imu_bridge_topic,
            "socket_path": imu_socket, "reliability": "reliable",
            "depth": 20, "stats_period": 10.0}])

    # restamp_imu reads the hop's output when the hop is active, else /IMU direct.
    restamp_imu = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_imu",
        output="screen", condition=IfCondition(imu_via_fastdds), parameters=[{
            "msg_type": "imu", "input_topic": imu_bridge_topic,
            "output_topic": "/imu", "frame_id": "imu_link",
            "mode": "offset", "reliability": "reliable"}])

    restamp_imu_direct = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_imu",
        output="screen", condition=UnlessCondition(imu_via_fastdds), parameters=[{
            "msg_type": "imu", "input_topic": imu_topic,
            "output_topic": "/imu", "frame_id": "imu_link",
            "mode": "offset", "reliability": sensor_reliability}])

    # /ODOM CROSS-RMW HOP — same shape as the /IMU hop above, and for the same
    # reason: the onboard state estimator (joint encoders + foot contact + IMU)
    # publishes over Fast-DDS from the same Foxy control stack as /IMU.
    #
    #   odom_uds_source [Fast-DDS]   /ODOM  --raw CDR-->  unix socket
    #   odom_uds_sink   [CycloneDDS] socket --raw CDR-->  /ODOM_fastdds
    #   restamp_odom    [CycloneDDS] /ODOM_fastdds -> /odom
    odom_source = Node(
        package="m20_bridge", executable="cdr_uds_relay.py",
        name="odom_uds_source", output="screen",
        condition=IfCondition(odom_via_fastdds),
        additional_env=_fastdds_env,
        parameters=[{
            "role": "source", "msg_type": "odometry", "topic": odom_topic,
            "socket_path": odom_socket, "reliability": sensor_reliability,
            "depth": 20, "stats_period": 10.0}])

    odom_sink = Node(
        package="m20_bridge", executable="cdr_uds_relay.py",
        name="odom_uds_sink", output="screen",
        condition=IfCondition(odom_via_fastdds),
        parameters=[{
            "role": "sink", "msg_type": "odometry", "topic": odom_bridge_topic,
            "socket_path": odom_socket, "reliability": "reliable",
            "depth": 20, "stats_period": 10.0}])

    # restamp_odom overrides frame_id/child_frame_id: the vendor message stamps
    # header.frame_id="map" (ITS OWN absolute reference, unrelated to this
    # stack's map frame from AMCL/the static map) and leaves child_frame_id
    # empty. Both are rewritten to this stack's odom -> base_link naming so
    # downstream (ekf_node's odom0) sees a normal odom message.
    restamp_odom = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_odom",
        output="screen", condition=IfCondition(odom_via_fastdds), parameters=[{
            "msg_type": "odometry", "input_topic": odom_bridge_topic,
            "output_topic": "/odom", "frame_id": "odom",
            "child_frame_id": "base_link", "mode": "offset",
            "reliability": "reliable"}])

    restamp_odom_direct = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_odom",
        output="screen", condition=UnlessCondition(odom_via_fastdds), parameters=[{
            "msg_type": "odometry", "input_topic": odom_topic,
            "output_topic": "/odom", "frame_id": "odom",
            "child_frame_id": "base_link", "mode": "offset",
            "reliability": sensor_reliability}])

    # /LIDAR/POINTS carries the ROBOT's clock, not this host's — use 'offset' to
    # rebase onto the GOS clock while preserving the 10 Hz spacing the LIO relies on.
    restamp_lidar = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_lidar",
        output="screen", parameters=[{
            "msg_type": "pointcloud2", "input_topic": lidar_topic,
            "output_topic": "/lidar/points", "frame_id": "lidar_link",
            "mode": "offset", "reliability": sensor_reliability}])

    restamp_lidar2 = Node(
        package="rl_deploy", executable="restamp_relay.py", name="restamp_lidar2",
        output="screen", condition=IfCondition(viz_lidar2), parameters=[{
            "msg_type": "pointcloud2", "input_topic": lidar2_topic,
            "output_topic": "/lidar/points2", "frame_id": "lidar_link",
            "mode": "arrival", "reliability": sensor_reliability}])

    # Front lidar body offset per manual §1.10 (320.28, 0, -13) mm.
    # TODO(calibration T3): verify origin/tilt via a ground-plane check in RViz.
    static_tf_lidar = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="lidar_link_static_tf",
        arguments=["0.32028", "0", "-0.013", "0", "0", "0",
                   "base_link", "lidar_link"])

    static_tf_imu = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="imu_link_static_tf",
        arguments=["0.0632", "-0.0268", "-0.0435", "0", "0", "0",
                   "base_link", "imu_link"])

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
        # the /JOINTS_DATA hop: convert in Fast-DDS, then relay to CycloneDDS
        joints_node, joints_source, joints_sink, joints_node_direct,
        # the /IMU hop: Fast-DDS source -> unix socket -> CycloneDDS sink
        imu_source, imu_sink,
        restamp_imu, restamp_imu_direct,
        # the /ODOM hop: Fast-DDS source -> unix socket -> CycloneDDS sink
        odom_source, odom_sink,
        restamp_odom, restamp_odom_direct,
        restamp_lidar, restamp_lidar2,
        cam1, cam2,
        static_tf_lidar, static_tf_imu, rsp,
    ])
