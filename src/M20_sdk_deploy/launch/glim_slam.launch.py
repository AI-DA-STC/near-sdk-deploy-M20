"""
GLIM SLAM Launch File for M20

Launches the GLIM SLAM node with M20_REAL config.
Topics (imu_topic/points_topic/image_topic) come from that config's
config_ros.json, not from this file: currently /imu, /lidar/points, /image.

Usage:
  ros2 launch rl_deploy glim_slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG_PATH = os.path.join(
    get_package_share_directory("rl_deploy"), "config", "glim", "M20_REAL"
)
# maps/ isn't an installed package resource, so there's no share-dir
# equivalent — this assumes invocation from the workspace root, as in
# every documented command in DEPLOYMENT.md.
DUMP_PATH = os.path.join(os.getcwd(), "maps")


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_slam',
            output='screen',
            parameters=[{
                'config_path': CONFIG_PATH,
                'dump_path': DUMP_PATH,
            }],
        ),
    ])
