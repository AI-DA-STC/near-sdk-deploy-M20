from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 8) Bridge Gazebo model poses → /gz/world_poses (ground truth for APE)
    bridge_gt_pose = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="M20_gt_pose_bridge",
        arguments=[
            "/world/Edifice/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
        ],
        remappings=[
            ("/world/Edifice/dynamic_pose/info", "/gz/world_poses")
        ],
        output="screen",
    )

    # 9) APE evaluator — compares AMCL+EKF estimate vs Gazebo ground truth
    ape_evaluator = Node(
        package="rl_deploy",
        executable="pose_ape_evaluator.py",
        name="pose_ape_evaluator",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([bridge_gt_pose, ape_evaluator])