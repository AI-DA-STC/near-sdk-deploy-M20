"""
GLIM SLAM Launch File for M20

Launches the GLIM SLAM node with M20_A config.
Subscribes to: /M20/IMU, /M20/LIDAR/VELODYNE

Usage:
  ros2 launch M20_sdk_deploy glim_slam.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from pyprojroot import here
import os

CONFIG_PATH = str(here() / "src" / "M20_sdk_deploy" / "config" / "glim" / "M20_A")
DUMP_PATH = str(here() / "maps")


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
