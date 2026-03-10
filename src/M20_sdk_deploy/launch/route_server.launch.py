from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pyprojroot import here

PROJECT_ROOT = here()
PACKAGE_PATH = PROJECT_ROOT / "src" / "M20_sdk_deploy"
NAV2_CONFIG = str(PACKAGE_PATH / "config" / "nav2" / "nav2_params.yaml")
DEFAULT_GRAPH = str(PROJECT_ROOT / "maps" / "graph.gml")


def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument(
            "graph_file",
            default_value=DEFAULT_GRAPH,
            description="Absolute path to the SWAGGER GML waypoint graph",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="TF frame for the output nav_msgs/Path",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock",
        ),
        DeclareLaunchArgument(
            "interpolation_resolution",
            default_value="0.05",
            description="Dense path step size in metres (default 5 cm)",
        ),
        DeclareLaunchArgument(
            "skeleton_only",
            default_value="false",
            description="Restrict Dijkstra to skeleton edges only",
        ),
    ]

    route_server = Node(
        package="rl_deploy",
        executable="route_server.py",
        name="route_server",
        output="screen",
        parameters=[
            NAV2_CONFIG,
            {
                "graph_file":               LaunchConfiguration("graph_file"),
                "frame_id":                 LaunchConfiguration("frame_id"),
                "use_sim_time":             LaunchConfiguration("use_sim_time"),
                "interpolation_resolution": LaunchConfiguration("interpolation_resolution"),
                "skeleton_only":            LaunchConfiguration("skeleton_only"),
            },
        ],
    )

    return LaunchDescription(declare_args + [route_server])
