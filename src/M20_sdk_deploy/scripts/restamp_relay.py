#!/usr/bin/env python3
"""
restamp_relay.py — subscribe to a topic, rebase header.stamp onto the LOCAL
clock, optionally rename topic frames, republish.

Why: a robot's hosts often run unsynchronized clocks (on the M20, AOS and GOS
are years apart). Any stream the nav stack consumes must share ONE clock or
tf2 dies with extrapolation errors. This relay lives in the *bridge* layer and
is the single place that clock differences are corrected, so nav_core only ever
sees local-clock, generically-named topics.

Two restamp modes:
  arrival  header.stamp := now().  Simplest; adds arrival jitter (~ms on-LAN)
           and DESTROYS the source inter-sample spacing (bursty DDS delivery
           makes consecutive dt collapse to {0, 0, 15 ms, ...}). Fine for pose
           streams into an EKF, WRONG for high-rate IMU feeding GLIM
           preintegration.
  offset   header.stamp := source_stamp + est_offset, where est_offset tracks
           the clock difference with a slewed min-filter (jumps down to any
           lower (arrival - source) observation = least-delayed packet, creeps
           up slowly to follow relative clock drift). This maps stamps onto the
           local clock WHILE PRESERVING the source dt, so 200 Hz IMU stays
           evenly spaced. A monotonic guard prevents non-increasing stamps.
           Use for /imu (GLIM) and /odom.

Parameters:
  msg_type        one of: pointcloud2 | odometry | jointstate | imu
  input_topic     source topic
  output_topic    destination topic
  frame_id        optional override of header.frame_id ('' = keep)
  child_frame_id  optional override (odometry only, '' = keep)
  reliability     'reliable' (default) or 'best_effort' — MUST match the
                  publisher or the subscription silently never matches (a
                  loud warning is logged if an incompatible peer is seen)
  mode            'arrival' (default) or 'offset'
  offset_drift_creep  seconds added to est_offset per message in offset mode
                  to follow slow clock drift (default 1e-6; ~ppm-scale)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# QoS-event callback classes moved packages across distros; try both.
try:
    from rclpy.event_handler import SubscriptionEventCallbacks, PublisherEventCallbacks
except ImportError:  # pragma: no cover - older distros
    from rclpy.qos_event import SubscriptionEventCallbacks, PublisherEventCallbacks

from sensor_msgs.msg import PointCloud2, JointState, Imu
from nav_msgs.msg import Odometry

MSG_TYPES = {
    'pointcloud2': PointCloud2,
    'odometry': Odometry,
    'jointstate': JointState,
    'imu': Imu,
}


class RestampRelay(Node):
    def __init__(self):
        super().__init__('restamp_relay')

        msg_type_name = self.declare_parameter('msg_type', '').value
        in_topic = self.declare_parameter('input_topic', '').value
        out_topic = self.declare_parameter('output_topic', '').value
        self._frame_id = self.declare_parameter('frame_id', '').value
        self._child_frame_id = self.declare_parameter('child_frame_id', '').value
        reliability = self.declare_parameter('reliability', 'reliable').value
        self._mode = self.declare_parameter('mode', 'arrival').value
        self._drift_creep = self.declare_parameter('offset_drift_creep', 1.0e-6).value

        if msg_type_name not in MSG_TYPES or not in_topic or not out_topic:
            raise RuntimeError(
                "restamp_relay needs msg_type (pointcloud2|odometry|jointstate|imu), "
                "input_topic and output_topic parameters")
        if self._mode not in ('arrival', 'offset'):
            raise RuntimeError("restamp_relay mode must be 'arrival' or 'offset'")
        msg_cls = MSG_TYPES[msg_type_name]

        qos = QoSProfile(depth=10)
        if reliability == 'best_effort':
            qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # offset-mode state
        self._est_offset = None      # seconds (arrival - source), slewed-min
        self._last_out = None        # seconds, monotonic guard

        # Loudly report QoS mismatches — otherwise a reliable/best_effort
        # disagreement makes the subscription match NOTHING with no error.
        sub_events = SubscriptionEventCallbacks(
            incompatible_qos=lambda ev: self.get_logger().error(
                f'INCOMPATIBLE QoS on subscription {in_topic}: '
                f'last_policy_kind={ev.last_policy_kind} — publisher and this '
                f'relay disagree (check reliability param). Receiving NOTHING.'))
        pub_events = PublisherEventCallbacks(
            incompatible_qos=lambda ev: self.get_logger().warn(
                f'INCOMPATIBLE QoS on publisher {out_topic}: '
                f'last_policy_kind={ev.last_policy_kind}'))

        self._pub = self.create_publisher(
            msg_cls, out_topic, qos, event_callbacks=pub_events)
        self._sub = self.create_subscription(
            msg_cls, in_topic, self._cb, qos, event_callbacks=sub_events)
        self.get_logger().info(
            f'restamp_relay[{self._mode}]: {in_topic} -> {out_topic} '
            f'({msg_type_name}, frame_id={self._frame_id or "<keep>"}, '
            f'reliability={reliability})')

    def _stamp_now(self):
        now = self.get_clock().now()
        return now, now.nanoseconds * 1e-9

    def _cb(self, msg):
        _, now_s = self._stamp_now()

        if self._mode == 'arrival':
            out_s = now_s
        else:  # offset: source_stamp + slewed-min clock offset
            src = msg.header.stamp
            src_s = src.sec + src.nanosec * 1e-9
            if src_s <= 0.0:
                out_s = now_s               # driver left stamp empty
            else:
                raw = now_s - src_s
                if self._est_offset is None or raw < self._est_offset:
                    self._est_offset = raw          # jump down to least-delayed
                else:
                    self._est_offset += self._drift_creep  # creep up to drift
                out_s = src_s + self._est_offset

        # monotonic guard
        if self._last_out is not None and out_s <= self._last_out:
            out_s = self._last_out + 1.0e-6
        self._last_out = out_s

        msg.header.stamp.sec = int(out_s)
        msg.header.stamp.nanosec = int(round((out_s - int(out_s)) * 1e9))
        if self._frame_id:
            msg.header.frame_id = self._frame_id
        if self._child_frame_id and hasattr(msg, 'child_frame_id'):
            msg.child_frame_id = self._child_frame_id
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = RestampRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
