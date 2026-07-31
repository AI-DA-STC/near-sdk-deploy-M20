#!/usr/bin/env python3
"""
cdr_uds_relay.py — hop one topic between two RMW implementations, via a Unix
datagram socket, inside a single container.

WHY THIS EXISTS
---------------
The M20's two DDS sensor streams are readable by DIFFERENT middlewares, and no
process can use both (RMW_IMPLEMENTATION is per-process). Measured on the robot
from a Humble container:

    topic            CycloneDDS 0.10      Fast-DDS 2.6
    /LIDAR/POINTS    9.3 Hz  ✅           matched nothing
    /IMU             matched nothing      200 Hz  ✅

Perfectly complementary, so one RMW always loses a sensor. The fix is two
processes with different RMWs and a NON-DDS channel between them. This script is
that channel: `--role source` subscribes under one RMW and writes raw CDR to a
Unix socket; `--role sink` reads the socket and republishes under the other.

WHICH STREAM GETS HOPPED, AND WHY IT IS THE IMU
-----------------------------------------------
The container's primary RMW is CycloneDDS, so /LIDAR/POINTS (~12 MB/s) stays
native and never touches this relay. Only /IMU is hopped, at ~300 B x 200 Hz =
~60 KB/s — about 200x less traffic through the socket. Hopping the cloud instead
would work but would push megabytes per scan through Python for no reason.

RAW CDR, SOCK_DGRAM
-------------------
The payload is the serialized CDR buffer (encapsulation header included), taken
straight from the wire and put back unchanged, so nothing is decoded here and
header.stamp still carries the ROBOT's clock for restamp_relay to correct. Both
ends are Humble, so the CDR is byte-identical by construction.

SOCK_DGRAM (not SOCK_STREAM) because one datagram == one message: no length
prefixes, no partial reads, no reconnect logic. The cost is a per-datagram size
ceiling (the kernel's socket buffer, ~200 KB by default). Fine for an Imu, NOT
fine for a point cloud — a warning is logged if a message ever gets close.

The source socket is NON-BLOCKING on purpose: if the sink stalls, we drop the
sample and count it rather than blocking the DDS callback and backing pressure
up into the robot's publisher.

Parameters
----------
  role         'source' (DDS -> socket) or 'sink' (socket -> DDS)
  topic        source: topic to subscribe; sink: topic to publish
  msg_type     imu | pointcloud2 | odometry | jointstate
  socket_path  filesystem path of the Unix datagram socket (same container)
  reliability  DDS QoS on this side: reliable (default) | best_effort
  depth        KEEP_LAST depth (default 20 — an IMU's history matters to GLIM)
  stats_period seconds between throughput reports (0 disables)
"""

import os
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    from rclpy.event_handler import (SubscriptionEventCallbacks,
                                     PublisherEventCallbacks)
except ImportError:  # pragma: no cover - Foxy / Galactic
    from rclpy.qos_event import (SubscriptionEventCallbacks,
                                 PublisherEventCallbacks)

from sensor_msgs.msg import Imu, PointCloud2, JointState
from nav_msgs.msg import Odometry

MSG_TYPES = {
    'imu': Imu,
    'pointcloud2': PointCloud2,
    'odometry': Odometry,
    'jointstate': JointState,
}

# Above this, SOCK_DGRAM starts failing against default socket buffers. An Imu is
# ~300 B, so hitting this means the relay was pointed at the wrong topic.
BIG_MSG_WARN = 64 * 1024
RCVBUF = 4 * 1024 * 1024


class CdrUdsRelay(Node):
    def __init__(self):
        super().__init__('cdr_uds_relay')

        self._role = self.declare_parameter('role', '').value
        topic = self.declare_parameter('topic', '').value
        msg_type_name = self.declare_parameter('msg_type', '').value
        self._path = self.declare_parameter('socket_path', '').value
        reliability = self.declare_parameter('reliability', 'reliable').value
        depth = self.declare_parameter('depth', 20).value
        stats_period = self.declare_parameter('stats_period', 10.0).value

        if self._role not in ('source', 'sink'):
            raise RuntimeError("cdr_uds_relay: role must be 'source' or 'sink'")
        if msg_type_name not in MSG_TYPES or not topic or not self._path:
            raise RuntimeError(
                'cdr_uds_relay needs topic, socket_path and msg_type '
                '(imu|pointcloud2|odometry|jointstate)')
        msg_cls = MSG_TYPES[msg_type_name]

        qos = QoSProfile(depth=depth)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = (ReliabilityPolicy.BEST_EFFORT
                           if reliability == 'best_effort'
                           else ReliabilityPolicy.RELIABLE)

        self._n = 0            # messages moved since the last report
        self._bytes = 0
        self._drops = 0        # sink not ready / socket full
        self._total = 0
        self._warned_big = False
        self._stats_period = stats_period or 10.0
        self._last_report = time.monotonic()
        self._rmw = os.environ.get('RMW_IMPLEMENTATION', '<default>')

        if self._role == 'source':
            self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            # Never block the DDS callback: drop and count instead.
            self._sock.setblocking(False)
            events = SubscriptionEventCallbacks(
                incompatible_qos=lambda ev: self.get_logger().error(
                    f'INCOMPATIBLE QoS subscribing {topic}: '
                    f'last_policy_kind={ev.last_policy_kind}. Receiving NOTHING '
                    f'— fix the reliability param; this is not a transport fault.'))
            self._sub = self.create_subscription(
                msg_cls, topic, self._on_dds, qos, raw=True,
                event_callbacks=events)
            self.create_timer(self._stats_period, self._report)
            self.get_logger().info(
                f'cdr_uds_relay[source, {self._rmw}]: {topic} -> {self._path} '
                f'({msg_type_name}, raw CDR, {reliability})')
        else:
            # Bind first so a restarting source finds the socket already there.
            if os.path.exists(self._path):
                os.unlink(self._path)          # stale socket from a previous run
            self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF)
            self._sock.bind(self._path)
            os.chmod(self._path, 0o666)
            self._sock.settimeout(0.5)         # so Ctrl-C is responsive
            events = PublisherEventCallbacks(
                incompatible_qos=lambda ev: self.get_logger().warn(
                    f'INCOMPATIBLE QoS publishing {topic}: '
                    f'last_policy_kind={ev.last_policy_kind}'))
            self._pub = self.create_publisher(
                msg_cls, topic, qos, event_callbacks=events)
            self.get_logger().info(
                f'cdr_uds_relay[sink, {self._rmw}]: {self._path} -> {topic} '
                f'({msg_type_name}, raw CDR, {reliability})')

    # ---- source side -------------------------------------------------------
    def _on_dds(self, raw):
        n = len(raw)
        if n > BIG_MSG_WARN and not self._warned_big:
            self._warned_big = True
            self.get_logger().warn(
                f'{n} B message through a SOCK_DGRAM relay — near the socket '
                f'buffer ceiling. This relay is meant for small, high-rate '
                f'topics like /IMU; large clouds should stay on their native RMW.')
        try:
            self._sock.sendto(raw, self._path)
            self._n += 1
            self._bytes += n
            self._total += 1
        except (FileNotFoundError, ConnectionRefusedError):
            self._drops += 1        # sink not up yet
        except BlockingIOError:
            self._drops += 1        # sink too slow; dropping beats blocking DDS

    # ---- sink side ---------------------------------------------------------
    def pump(self):
        """Blocking read loop. The sink needs no executor: it has no
        subscriptions or timers, so spinning would only add latency."""
        while rclpy.ok():
            try:
                raw = self._sock.recv(RCVBUF)
            except socket.timeout:
                self._maybe_report()
                continue
            except OSError:
                break
            self._pub.publish(raw)
            self._n += 1
            self._bytes += len(raw)
            self._total += 1
            self._maybe_report()

    def _maybe_report(self):
        if time.monotonic() - self._last_report >= self._stats_period:
            self._report()

    def _report(self):
        now = time.monotonic()
        dt = max(now - self._last_report, 1e-6)
        self._last_report = now
        if self._n == 0:
            self.get_logger().warn(
                f'[{self._role}] 0 msg in {dt:.0f}s (total {self._total}, '
                f'{self._drops} dropped). '
                + ('Is the publisher up, and is this process really using the '
                   'RMW that can see it? ' if self._role == 'source'
                   else 'Is the source process alive and pointed at the same '
                        'socket_path? '))
        else:
            self.get_logger().info(
                f'[{self._role}] {self._n / dt:.1f} Hz, '
                f'{self._bytes / 1e3 / dt:.1f} kB/s '
                f'(total {self._total}, {self._drops} dropped)')
        self._n = 0
        self._bytes = 0

    def close(self):
        try:
            self._sock.close()
        except OSError:
            pass
        if self._role == 'sink' and self._path and os.path.exists(self._path):
            try:
                os.unlink(self._path)
            except OSError:
                pass


def main():
    rclpy.init()
    node = CdrUdsRelay()
    try:
        if node._role == 'sink':
            node.pump()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
