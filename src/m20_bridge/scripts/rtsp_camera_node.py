#!/usr/bin/env python3
"""
rtsp_camera_node.py — pull an RTSP H.264 stream from the M20 and republish it as
a ROS2 sensor_msgs/Image topic.

The M20's wide-angle cameras are served over RTSP on the AOS (vendor manual
Appendix 3): rtsp://10.21.31.103:8554/video1 (front) and .../video2 (rear).
Unlike the DDS sensor topics, RTSP is a plain TCP stream, so this node runs
happily on the GOS in the m20_bridge container and just needs network reach to
the AOS on :8554.

One instance per camera; parameterise rtsp_url / output_topic / frame_id in the
launch file. Frames are read in a background thread (cv2.VideoCapture.read()
blocks) and the stream auto-reconnects if it drops.

Params:
  rtsp_url       full RTSP URL (required), e.g. rtsp://10.21.31.103:8554/video1
  output_topic   topic to publish on (default /camera)
  frame_id       header.frame_id (default camera_link)
  fps            publish throttle in Hz (default 15.0; <=0 = every decoded frame)
"""

import os
import threading
import time

# Force RTSP-over-TCP (survives lossy links far better than the UDP default) and
# a 5 s socket timeout so a dead stream fails read() instead of blocking forever.
# MUST be set before cv2 opens the capture.
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                      "rtsp_transport;tcp|stimeout;5000000")

import cv2  # noqa: E402  (after the env var above)
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402


class RtspCamera(Node):
    def __init__(self):
        super().__init__("rtsp_camera")
        self._url = self.declare_parameter("rtsp_url", "").value
        out_topic = self.declare_parameter("output_topic", "/camera").value
        self._frame_id = self.declare_parameter("frame_id", "camera_link").value
        self._fps = float(self.declare_parameter("fps", 15.0).value)
        if not self._url:
            raise RuntimeError("rtsp_camera needs the rtsp_url parameter")

        # SensorData-style QoS: best_effort + shallow queue. Video wants the
        # latest frame, never a retransmitted stale one, and this keeps big
        # Image samples off the reliable path. Viewers (RViz/rqt) must also
        # select best_effort or they will see nothing.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(Image, out_topic, qos)
        self._bridge = CvBridge()
        self._min_dt = 1.0 / self._fps if self._fps > 0 else 0.0

        self.get_logger().info(f"rtsp_camera: {self._url} -> {out_topic} "
                               f"(frame_id={self._frame_id}, fps={self._fps})")
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open(self):
        cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # minimise latency
        except Exception:
            pass
        return cap

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _loop(self):
        cap = self._open()
        last_pub = 0.0
        while rclpy.ok() and not self._stop:
            if not cap.isOpened():
                self.get_logger().warn("stream not open; reconnecting in 1 s")
                cap.release()
                time.sleep(1.0)
                cap = self._open()
                continue
            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warn("frame read failed; reconnecting in 1 s")
                cap.release()
                time.sleep(1.0)
                cap = self._open()
                continue
            # fps throttle: keep decoding (drains the buffer -> low latency) but
            # only publish at the target rate.
            now = self._now_s()
            if self._min_dt and (now - last_pub) < self._min_dt:
                continue
            last_pub = now

            # Camera clock is unknown/unsynced; stamp with the local (GOS) clock
            # so downstream tf2 stays consistent with the other bridged streams.
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            self._pub.publish(msg)
        cap.release()

    def destroy_node(self):
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = RtspCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
