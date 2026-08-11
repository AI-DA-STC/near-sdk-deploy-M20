#!/usr/bin/env python3
"""
m20_joints_node.py — drdds/JointsData on /JOINTS_DATA -> sensor_msgs/JointState.

Replaces the joint-state half of m20_udp_node.py. The UDP Inspection Protocol
(1002/4 MotorStatus) carries the same 16 motor angles, but only as a JSON blob
at the status rate and with no velocity or torque; /JOINTS_DATA is the control
stack's native 200 Hz publication and carries all three.

THE CONVENTION CONVERSION IS THE POINT OF THIS NODE. The robot reports raw motor
angles, which are NOT the URDF's joint angles: the vendor mirrors its zero and
sign per leg, and several joints are a full turn away from the URDF's zero. The
conversion is the exact inverse of what the Gazebo controller applies when it
fakes this same topic for simulation, in rl_deploy's
interface/robot/simulation/gazebo_controller_ros2.py:

    (sim, URDF -> robot)   pub_pos = (q_urdf - POS_OFFSET_RAD) * JOINT_DIR
    (here, robot -> URDF)  q_urdf  = wrap(q_vendor * JOINT_DIR + POS_OFFSET_RAD)

JOINT_DIR entries are +-1, so multiplying inverts them. The wrap to [-pi, pi] is
required, not cosmetic: several offsets exceed 180 deg (fl_hipy is +229 deg), so
without it the result lands a full turn outside the URDF's <limit> and RViz
renders the robot inverted. Every revolute limit in M20.urdf is inside +-pi, so
wrapping can never discard a legal pose.

Validated against two captured postures on the real robot:

    posture    body height   feet coplanar   feet under their hips
    standing   0.582 m       12 mm           within 25 mm
    sitting    0.167 m       23 mm           within 25 mm

with the standing figure agreeing to 15 mm with the Height the robot itself
reports in UDP 1002/4 MotionStatus.

Velocity and torque take the sign flip but NOT the offset, matching the same
reference (pub_vel = dq * JOINT_DIR).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

from drdds.msg import JointsData

DOF = 16

# URDF joint names in the robot's JointsData array order.
JOINT_NAMES = [
    'fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint', 'fl_wheel_joint',
    'fr_hipx_joint', 'fr_hipy_joint', 'fr_knee_joint', 'fr_wheel_joint',
    'hl_hipx_joint', 'hl_hipy_joint', 'hl_knee_joint', 'hl_wheel_joint',
    'hr_hipx_joint', 'hr_hipy_joint', 'hr_knee_joint', 'hr_wheel_joint',
]

# Vendor calibration, verbatim from gazebo_controller_ros2.py so the two stay
# in step. hipx/hipy/knee/wheel per leg, legs in fl, fr, hl, hr order.
JOINT_DIR = [1.0, 1.0, -1.0, 1.0,
             1.0, -1.0, 1.0, -1.0,
             -1.0, 1.0, -1.0, 1.0,
             -1.0, -1.0, 1.0, -1.0]

POS_OFFSET_DEG = [-25.0, 229.0, 160.0, 0.0,
                  25.0, -131.0, -200.0, 0.0,
                  -25.0, -229.0, -160.0, 0.0,
                  25.0, 131.0, 200.0, 0.0]


def _wrap(a):
    """Fold into [-pi, pi]. Safe here: every revolute limit in M20.urdf is
    inside +-pi, so no legal pose can be wrapped away."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class M20JointsNode(Node):
    def __init__(self):
        super().__init__('m20_joints_node')

        p = self.declare_parameter
        self._in = p('joints_topic', '/JOINTS_DATA').value
        self._out = p('output_topic', '/joint_states').value
        # The robot's clock is not this host's — every other stream in this
        # bridge is restamped for exactly that reason. Joint states feed
        # robot_state_publisher, which wants stamps on the local timeline.
        self._use_source_stamp = p('use_source_stamp', False).value
        # Exposed so a recalibration needs no rebuild.
        self._dir = list(p('joint_dir', JOINT_DIR).value)
        self._off = [d / 180.0 * math.pi
                     for d in p('pos_offset_deg', POS_OFFSET_DEG).value]
        reliability = p('reliability', 'reliable').value

        if len(self._dir) != DOF or len(self._off) != DOF:
            raise ValueError(
                f'joint_dir and pos_offset_deg must both have {DOF} entries, '
                f'got {len(self._dir)} and {len(self._off)}')

        qos = QoSProfile(
            depth=20,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(ReliabilityPolicy.RELIABLE if reliability == 'reliable'
                         else ReliabilityPolicy.BEST_EFFORT))

        self._pub = self.create_publisher(JointState, self._out, qos)
        self.create_subscription(JointsData, self._in, self._cb, qos)

        self._n = 0
        self._warned_short = False
        self.get_logger().info(
            f'{self._in} (drdds/JointsData) -> {self._out} '
            f'(sensor_msgs/JointState), {reliability}, '
            f'vendor convention -> URDF convention')

    def _cb(self, msg):
        joints = msg.data.joints_data
        if len(joints) < DOF:
            if not self._warned_short:
                self._warned_short = True
                self.get_logger().error(
                    f'{self._in} carries {len(joints)} joints, expected {DOF} '
                    f'— check the robot firmware version; not publishing')
            return

        js = JointState()
        if self._use_source_stamp:
            js.header.stamp = msg.header.stamp
        else:
            js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = [_wrap(joints[i].position * self._dir[i] + self._off[i])
                       for i in range(DOF)]
        js.velocity = [joints[i].velocity * self._dir[i] for i in range(DOF)]
        js.effort = [joints[i].torque * self._dir[i] for i in range(DOF)]
        self._pub.publish(js)

        self._n += 1
        if self._n == 1:
            self.get_logger().info(
                'first frame, URDF convention: ' + '  '.join(
                    f'{n.split("_joint")[0]}={p:+.3f}'
                    for n, p in zip(js.name, js.position)
                    if not n.endswith('wheel_joint')))


def main():
    rclpy.init()
    node = M20JointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
