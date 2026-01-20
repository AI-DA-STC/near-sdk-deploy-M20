#!/usr/bin/python3
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64

# Import custom messages
from drdds.msg import (
    ImuData, JointsData, JointsDataCmd, MetaType, 
    ImuDataValue, JointsDataValue, JointData, JointDataCmd
)


MODEL_NAME = "M20"

# Calibration parameters (for sim-to-real consistency)
JOINT_DIR = np.array([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1], dtype=np.float32)
POS_OFFSET_DEG = np.array([-25, 229, 160, 0, 25, -131, -200, 0, -25, -229, -160, 0, 25, 131, 200, 0], dtype=np.float32)
POS_OFFSET_RAD = POS_OFFSET_DEG / 180.0 * np.pi

# Joint names in order (16 DOF)
JOINT_NAMES = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint'
]


class GazeboControllerNode(Node):
    def __init__(self):
        super().__init__('gazebo_controller')
        
        self.dof_num = 16
        
        # Control command buffers
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        
        # Current state buffers
        self.joint_positions = np.zeros(self.dof_num, dtype=np.float32)
        self.joint_velocities = np.zeros(self.dof_num, dtype=np.float32)
        self.joint_efforts = np.zeros(self.dof_num, dtype=np.float32)
        
        # IMU state
        self.imu_orientation = Quaternion()
        self.imu_angular_velocity = np.zeros(3, dtype=np.float32)
        self.imu_linear_acceleration = np.zeros(3, dtype=np.float32)
        
        self.timestamp = 0.0
        self.joints_received = False
        self.imu_received = False
        
        self.get_logger().info(f"[INFO] Gazebo Controller initialized, dof = {self.dof_num}")
        
        # Publishers for custom messages (same as MuJoCo)
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)
        
        self.get_logger().info("[INFO] Publishing to /IMU_DATA and /JOINTS_DATA")
        
        # Subscriber for custom joint commands (same as MuJoCo)
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            '/JOINTS_CMD',
            self._cmd_callback,
            50
        )
        
        # Gazebo bridge subscribers (receive state from Gazebo)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.get_logger().info("[INFO] Subscribed to /joint_states")
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
            10
        )
        self.get_logger().info("[INFO] Subscribed to /imu/data")
        
        # Gazebo bridge publishers (send force commands to each joint)
        self.joint_force_pubs = []
        for joint_name in JOINT_NAMES:
            pub = self.create_publisher(
                Float64,
                f'/model/M20/joint/{joint_name}/cmd_force',
                10
            )
            self.joint_force_pubs.append(pub)
        
        # Timer to publish custom messages at 200 Hz
        self.publish_timer = self.create_timer(0.005, self._publish_robot_state)
        
        # Timer to send commands to Gazebo at 1000 Hz
        self.control_timer = self.create_timer(0.001, self._send_joint_commands)
        
        self.get_logger().info("[INFO] Gazebo Controller ready")
    
    def _cmd_callback(self, msg: JointsDataCmd):
        """Convert received (published) positions/velocities to internal (raw)"""
        if len(msg.data.joints_data) != 16:
            self.get_logger().warn("Received JointsDataCmd with incorrect number of joints")
            return
        
        pub_pos = np.zeros(self.dof_num, dtype=np.float32)
        pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp if not np.isnan(joint_cmd.kp) else 0.0
            self.kd_cmd[i] = joint_cmd.kd if not np.isnan(joint_cmd.kd) else 0.0
            pub_pos[i] = joint_cmd.position if not np.isnan(joint_cmd.position) else 0.0
            pub_vel[i] = joint_cmd.velocity if not np.isnan(joint_cmd.velocity) else 0.0
            self.tau_ff[i] = joint_cmd.torque if not np.isnan(joint_cmd.torque) else 0.0
        
        # Convert: raw = published * dir + offset_rad
        self.pos_cmd.flat = pub_pos * JOINT_DIR + POS_OFFSET_RAD
        self.vel_cmd.flat = pub_vel * JOINT_DIR
    
    def _joint_state_callback(self, msg: JointState):
        """Receive joint states from Gazebo"""
        if not self.joints_received:
            self.get_logger().info(f"[INFO] Received first joint state message with {len(msg.name)} joints")
            self.joints_received = True
        
        # Map joint states to our order
        for i, name in enumerate(JOINT_NAMES):
            try:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    self.joint_velocities[i] = msg.velocity[idx]
                if len(msg.effort) > idx:
                    self.joint_efforts[i] = msg.effort[idx]
            except (ValueError, IndexError):
                pass
    
    def _imu_callback(self, msg: Imu):
        """Receive IMU data from Gazebo"""
        if not self.imu_received:
            self.get_logger().info("[INFO] Received first IMU message")
            self.imu_received = True
            
        self.imu_orientation = msg.orientation
        self.imu_angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=np.float32)
        self.imu_linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=np.float32)
    
    def _send_joint_commands(self):
        """Calculate and send joint torque commands to Gazebo"""
        # Current joint state
        q = self.joint_positions.reshape(-1, 1)
        dq = self.joint_velocities.reshape(-1, 1)
        
        # PD control law with feedforward torque
        input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            self.kd_cmd * (self.vel_cmd - dq) +
            self.tau_ff
        )
        
        # Check for NaN or Inf values in inputs
        if np.any(np.isnan(self.pos_cmd)) or np.any(np.isinf(self.pos_cmd)):
            self.get_logger().warn(f"Invalid pos_cmd detected: {self.pos_cmd.flatten()[:4]}")
            self.pos_cmd = np.nan_to_num(self.pos_cmd, nan=0.0, posinf=0.0, neginf=0.0)
        
        if np.any(np.isnan(self.kp_cmd)) or np.any(np.isinf(self.kp_cmd)):
            self.get_logger().warn(f"Invalid kp_cmd detected: {self.kp_cmd.flatten()[:4]}")
            self.kp_cmd = np.nan_to_num(self.kp_cmd, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Check for NaN or Inf values in output
        if np.any(np.isnan(input_tq)) or np.any(np.isinf(input_tq)):
            self.get_logger().error(
                f"Invalid torque detected! "
                f"pos_cmd[0]={self.pos_cmd[0,0]:.3f}, q[0]={q[0,0]:.3f}, "
                f"kp[0]={self.kp_cmd[0,0]:.3f}, kd[0]={self.kd_cmd[0,0]:.3f}"
            )
            input_tq = np.nan_to_num(input_tq, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Publish individual force commands for each joint
        for i, pub in enumerate(self.joint_force_pubs):
            force_msg = Float64()
            force_msg.data = float(input_tq[i, 0])
            pub.publish(force_msg)
        
        # Store computed torques for state publishing
        self.joint_efforts = input_tq.flatten()
    
    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q.w, q.x, q.y, q.z
        
        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        
        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        
        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        
        return np.array([roll, pitch, yaw], dtype=np.float32)
    
    def _publish_robot_state(self):
        """Publish robot state in custom message format (200 Hz)"""
        self.timestamp = self.get_clock().now().nanoseconds / 1e9
        
        # ----- IMU -----
        rpy_rad = self.quaternion_to_euler(self.imu_orientation)
        rpy_deg = [angle * (180.0 / np.pi) for angle in rpy_rad]
        
        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(self.imu_angular_velocity[0])
        imu_msg.data.omega_y = float(self.imu_angular_velocity[1])
        imu_msg.data.omega_z = float(self.imu_angular_velocity[2])
        imu_msg.data.acc_x = float(self.imu_linear_acceleration[0])
        imu_msg.data.acc_y = float(self.imu_linear_acceleration[1])
        imu_msg.data.acc_z = float(self.imu_linear_acceleration[2])
        self.imu_pub.publish(imu_msg)
        
        # ----- Joints -----
        q = self.joint_positions
        dq = self.joint_velocities
        tau = self.joint_efforts
        
        # Convert raw to published: published = (raw - offset_rad) * dir
        pub_pos = (q - POS_OFFSET_RAD) * JOINT_DIR
        pub_vel = dq * JOINT_DIR
        pub_tau = tau * JOINT_DIR
        
        # Sanitize values before publishing
        pub_pos = np.nan_to_num(pub_pos, nan=0.0, posinf=0.0, neginf=0.0)
        pub_vel = np.nan_to_num(pub_vel, nan=0.0, posinf=0.0, neginf=0.0)
        pub_tau = np.nan_to_num(pub_tau, nan=0.0, posinf=0.0, neginf=0.0)
        
        joints_msg = JointsData()
        joints_msg.header = MetaType()
        joints_msg.header.frame_id = 0
        stamp = Time()
        sec = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(self.dof_num)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]
            joint.name = b'    '  # Dummy name (four spaces as bytes)
            joint.data_id = 0
            joint.status_word = 1  # Normal
            joint.position = float(pub_pos[i])
            joint.torque = float(pub_tau[i])
            joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0  # Dummy normal temp
            joint.driver_temp = 45.0  # Dummy normal temp
        self.joints_pub.publish(joints_msg)


def main(args=None):
    rclpy.init(args=args)
    controller_node = GazeboControllerNode()
    
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
