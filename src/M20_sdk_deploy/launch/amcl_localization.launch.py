from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pyprojroot import here

PROJECT_ROOT = here()
PACKAGE_PATH = PROJECT_ROOT / "src" / "M20_sdk_deploy"
NAV2_CONFIG = str(PACKAGE_PATH / "config" / "nav2" / "amcl_params.yaml")
EKF_CONFIG = str(PACKAGE_PATH / "config" / "nav2" / "ekf_params.yaml")


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")

    declare_args = [
        DeclareLaunchArgument(
            "map",
            default_value=str(PROJECT_ROOT / "maps" / "edifice_SLAM_v0_2d.yaml"),
            description="Path to 2D map .yaml file",
        ),
    ]

    # 2) Convert 3D PointCloud2 -> 2D LaserScan for AMCL
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
            "angle_increment": 0.00287,  # ~2187 samples over 360 deg
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

    # 1) rf2o scan-matching odometry (LaserScan -> Odometry for EKF)
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
            "init_pose_from_topic": "",  # empty = start at (0,0,0) immediately
            "publish_tf": False,  # EKF owns the odom -> base_link TF
            "freq": 10.0,
        }],
    )

    # 3) Map server - serves the 2D occupancy grid
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[
            NAV2_CONFIG,
            {"yaml_filename": map_yaml},
        ],
    )

    # 4) AMCL - publishes map -> odom transform
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[NAV2_CONFIG],
    )

    # 5) Lifecycle manager - activates map_server + amcl
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[NAV2_CONFIG],
    )

    # 6) EKF - fuses LiDAR odom + IMU to publish odom -> base_link
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[EKF_CONFIG],
    )

    # 7) Static TFs for Gazebo frame names
    # Gazebo bridge uses "M20/base_link/<sensor>" frame_ids instead of just "<sensor>".
    # Without these, pointcloud_to_laserscan and EKF silently drop all messages.
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

    return LaunchDescription(
        declare_args + [
            static_tf_gazebo_velodyne,
            static_tf_gazebo_imu,
            pointcloud_to_laserscan,
            rf2o_laser_odometry,
            map_server,
            amcl,
            lifecycle_manager,
            ekf_node
        ]
    )
