from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def remove_entity_action(world_name_str: str, entity_name: str):
    """
    Create an ExecuteProcess action that removes an entity by name from Gazebo.
    Uses the Gazebo Transport service: /world/<world>/remove
    """
    service = f"/world/{world_name_str}/remove"

    # Protobuf text format request for ignition.msgs.Entity
    # Type enum: MODEL is typically what you want for world models.
    req = f'name: "{entity_name}" type: MODEL'

    return ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", service,
            "--reqtype", "ignition.msgs.Entity",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "3000",
            "--req", req,
        ],
        output="screen",
    )


def generate_launch_description():
    world_name = LaunchConfiguration("world_name")
    ign_world = LaunchConfiguration("ign_world")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    declare_args = [
        DeclareLaunchArgument("world_name", default_value="Edifice"),
        DeclareLaunchArgument(
            "ign_world",
            default_value='https://fuel.ignitionrobotics.org/1.0/OpenRobotics/worlds/Edifice demo',
            description='Exactly what you pass to: ign gazebo -v 4 "<IGN_WORLD>"',
        ),
        DeclareLaunchArgument("robot_name", default_value="M20"),
        DeclareLaunchArgument(
            "robot_sdf",
            default_value="/home/krishna/Workspace/near-sdk-deploy-M20/src/M20_sdk_deploy/model/M20_urdf/urdf/M20.sdf",
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.57"),  # Higher spawn to prevent immediate fall
    ]

    # 1) Start Gazebo with auto-run (not paused)
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-v", "4", "-r", ign_world],  # -r flag starts simulation running
        output="screen",
    )

    # 2) Spawn robot with ros_gz_sim create
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", robot_name,
            "-file", robot_sdf,
            "-x", x,
            "-y", y,
            "-z", z,
        ],
    )

    # Remove unwanted entities from the world
    remove_franka = remove_entity_action("Edifice", "Panda")
    remove_table = remove_entity_action("Edifice", "Reflective table")
    
    # Wait 8 seconds to remove entities, then spawn robot at 10 seconds
    delayed_remove = TimerAction(period=8.0, actions=[remove_franka, remove_table])
    delayed_spawn = TimerAction(period=12.0, actions=[spawn_robot])

    # 3) Bridge joint states from Gazebo to ROS2
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/Edifice/model/M20/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/Edifice/model/M20/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # 4) Bridge IMU data from Gazebo to ROS2
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/M20/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        remappings=[
            ('/model/M20/link/base_link/sensor/imu_sensor/imu', '/imu/data')
        ],
        output='screen'
    )

    # 5) Bridge joint force commands for each joint
    bridge_joints = []
    joint_names = [
        'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
        'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
        'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
        'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
    ]
    
    for joint_name in joint_names:
        bridge_joints.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/model/M20/joint/{joint_name}/cmd_force@std_msgs/msg/Float64]gz.msgs.Double'
                ],
                output='screen'
            )
        )

    # 6) Controller node - Python script
    controller_node = Node(
        package='rl_deploy',
        executable='gazebo_controller_ros2.py',
        output='screen',
        name='gazebo_controller'
    )

    # Start bridges and controller after robot spawns
    delayed_bridges_controller = TimerAction(
        period=15.0, 
        actions=[bridge_joint_states, bridge_imu] + bridge_joints + [controller_node]
    )

    return LaunchDescription(declare_args + [gazebo, delayed_remove, delayed_spawn, delayed_bridges_controller])
