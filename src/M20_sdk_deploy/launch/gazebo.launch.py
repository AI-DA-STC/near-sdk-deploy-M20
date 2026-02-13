"""
Single Robot Gazebo Launch File

Launches a single M20 robot in the Gazebo world with configurable robot name.

Usage:
  # Default robot name (M20):
  ros2 launch rl_deploy gazebo.launch.py

  # With specific robot name:
  ros2 launch rl_deploy gazebo.launch.py robot_name:=M20_A
  ros2 launch rl_deploy gazebo.launch.py robot_name:=M20_B

  # With custom spawn position:
  ros2 launch rl_deploy gazebo.launch.py robot_name:=M20_A x:=-2.0 y:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

# Path to the URDF file for robot_state_publisher
URDF_FILE = "/home/krishna/Workspace/near-sdk-deploy-M20/src/M20_sdk_deploy/model/M20_urdf/urdf/M20.urdf"

# Path to the model directory containing Depot_simple
MODEL_PATH = "/home/krishna/Workspace/near-sdk-deploy-M20/src/M20_sdk_deploy/model"
# Path to the simplified world file
WORLD_FILE = "/home/krishna/Workspace/near-sdk-deploy-M20/src/M20_sdk_deploy/model/Edifice_simple/edifice_simple.sdf"
# Path to the robot SDF
ROBOT_SDF = "/home/krishna/Workspace/near-sdk-deploy-M20/src/M20_sdk_deploy/model/M20_urdf/urdf/M20.sdf"

# Joint names for bridge setup
JOINT_NAMES = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
]

# Sensor frame configurations (from M20.sdf)
# Format: (sensor_name, x, y, z, roll, pitch, yaw)
SENSOR_FRAMES = [
    ('front_lidar', 0.32028, 0, -0.013, 0, 0, 0),
    ('rear_lidar', -0.32028, 0, -0.013, 0, 0, 3.14159),
    ('imu_sensor', 0.0632, -0.0268, -0.0435, 0, 0, 0),
    ('front_camera', 0.37646, 0, 0.03738, 0, 0, 0),
    ('rear_camera', -0.37646, 0, 0.03738, 0, 0, 3.14159),
]


def launch_setup(context, *args, **kwargs):
    """Setup function that has access to launch configurations as strings."""

    # Get launch configuration values as strings
    robot_name = LaunchConfiguration('robot_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)

    # Load URDF content for robot_state_publisher
    with open(URDF_FILE, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Get existing IGN_GAZEBO_RESOURCE_PATH and append our model path
    existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    new_resource_path = f"{MODEL_PATH}:{existing_path}" if existing_path else MODEL_PATH

    nodes_and_actions = []

    # 1) Set environment variable for Gazebo to find our custom models
    nodes_and_actions.append(SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=new_resource_path
    ))

    # 2) Start Gazebo with simplified world (auto-run, not paused)
    nodes_and_actions.append(ExecuteProcess(
        cmd=["ign", "gazebo", "-v", "4", "-r", WORLD_FILE],
        output="screen",
        additional_env={"IGN_GAZEBO_RESOURCE_PATH": new_resource_path},
    ))

    # 3) GPU monitoring - shows GPU utilization, VRAM, and temperature every 3 seconds
    nodes_and_actions.append(ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 8 && while true; do "
            "echo '' && "
            "echo '[GPU STATS] '$(date '+%H:%M:%S')' -------------------------------------------' && "
            "nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits | "
            "awk -F', ' '{printf \"  GPU Util: %3d%% | Mem Util: %3d%% | VRAM: %s/%s MiB | Temp: %s C\\n\", $1, $2, $3, $4, $5}' && "
            "sleep 3; "
            "done"
        ],
        output="screen",
    ))

    # 4) Gazebo stats monitor - shows real-time factor and sim time
    nodes_and_actions.append(ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 10 && while true; do "
            "ign topic -e -t /stats -n 1 2>/dev/null | "
            "grep -E '(real_time_factor|sim_time|iterations)' | head -6 | "
            "awk '/real_time_factor/ {rtf=$2} /sim_time.*sec:/ {st=$2} /iterations/ {iter=$2} "
            "END {if(rtf) printf \"[SIM STATS] RTF: %.2f | Iterations: %s\\n\", rtf, iter}' && "
            "sleep 3; "
            "done"
        ],
        output="screen",
    ))

    # 5) Spawn robot with ros_gz_sim create (delayed to let world load)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", robot_name,
            "-file", ROBOT_SDF,
            "-x", x,
            "-y", y,
            "-z", z,
        ],
    )
    nodes_and_actions.append(TimerAction(period=8.0, actions=[spawn_robot]))

    # --- Bridges and controllers (delayed to start after robot spawns) ---
    delayed_nodes = []

    # 6) Bridge joint states from Gazebo to ROS2
    delayed_nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_joint_state_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/joint_state', f'/{robot_name}/joint_states')
        ],
        output='screen'
    ))

    # 7) Bridge IMU data from Gazebo to ROS2
    delayed_nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_imu_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/imu_sensor/imu', f'/{robot_name}/IMU')
        ],
        output='screen'
    ))

    # 8) Bridge front LiDAR point cloud data from Gazebo to ROS2
    delayed_nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_front_lidar_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/front_lidar/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/front_lidar/scan/points', f'/{robot_name}/LIDAR/FRONT')
        ],
        output='screen'
    ))

    # 9) Bridge rear LiDAR point cloud data from Gazebo to ROS2
    delayed_nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_rear_lidar_bridge',
        arguments=[
            f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/rear_lidar/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        remappings=[
            (f'/world/{world_name}/model/{robot_name}/link/base_link/sensor/rear_lidar/scan/points', f'/{robot_name}/LIDAR/REAR')
        ],
        output='screen'
    ))

    # 10) Bridge joint force commands for each joint
    for joint_name in JOINT_NAMES:
        delayed_nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{robot_name}_{joint_name}_bridge',
            arguments=[
                f'/model/{robot_name}/joint/{joint_name}/cmd_force@std_msgs/msg/Float64]ignition.msgs.Double'
            ],
            output='screen'
        ))

    # 11) Controller node - Python script with parameters
    delayed_nodes.append(Node(
        package='rl_deploy',
        executable='gazebo_controller_ros2.py',
        output='screen',
        name='gazebo_controller',
        namespace=robot_name,
        parameters=[{
            'robot_name': robot_name,
            'world_name': world_name,
        }],
    ))

    # 12) Robot State Publisher - publishes TF transforms from URDF and joint states
    delayed_nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        namespace=robot_name,
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': f'{robot_name}/',
        }],
        remappings=[
            ('joint_states', f'/{robot_name}/joint_states'),
        ],
    ))

    # 13) Static transform publishers for sensor frames
    # These create TF frames for sensors that Gazebo references in sensor messages
    for sensor_name, x, y, z, roll, pitch, yaw in SENSOR_FRAMES:
        delayed_nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{robot_name}_{sensor_name}_tf',
            arguments=[
                '--x', str(x),
                '--y', str(y),
                '--z', str(z),
                '--roll', str(roll),
                '--pitch', str(pitch),
                '--yaw', str(yaw),
                '--frame-id', f'{robot_name}/base_link',
                '--child-frame-id', f'{robot_name}/base_link/{sensor_name}',
            ],
            output='screen'
        ))

    # Start bridges and controller after robot spawns
    nodes_and_actions.append(TimerAction(
        period=15.0,
        actions=delayed_nodes
    ))

    return nodes_and_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="Edifice"),
        DeclareLaunchArgument("robot_name", default_value="M20"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.57"),
        OpaqueFunction(function=launch_setup),
    ])
