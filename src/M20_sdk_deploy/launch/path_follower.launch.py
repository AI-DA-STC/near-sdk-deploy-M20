from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument(
            "max_speed",
            default_value="0.5",
            description="Maximum forward speed [m/s]",
        ),
        DeclareLaunchArgument(
            "angular_gain",
            default_value="1.0",
            description="Yaw rate gain: vyaw = angular_gain * heading_error",
        ),
        DeclareLaunchArgument(
            "lookahead_distance",
            default_value="1.5",
            description="Pure-pursuit lookahead distance [m]",
        ),
        DeclareLaunchArgument(
            "goal_tolerance",
            default_value="0.4",
            description="Distance from goal pose at which the robot stops [m]",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock",
        ),
    ]

    path_follower = Node(
        package="rl_deploy",
        executable="path_follower.py",
        name="path_follower",
        output="screen",
        parameters=[
            {
                "max_speed":          LaunchConfiguration("max_speed"),
                "angular_gain":       LaunchConfiguration("angular_gain"),
                "lookahead_distance": LaunchConfiguration("lookahead_distance"),
                "goal_tolerance":     LaunchConfiguration("goal_tolerance"),
                "use_sim_time":       LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(declare_args + [path_follower])
