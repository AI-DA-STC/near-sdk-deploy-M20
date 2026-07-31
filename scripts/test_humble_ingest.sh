#!/usr/bin/env bash
# =============================================================================
# test_humble_ingest.sh — can the HUMBLE container read the vendor's
# /LIDAR/POINTS directly, and under which RMW + QoS?
#
# THIS IS THE MEASUREMENT THAT PINNED THE RMW. Result on the M20:
#
#   A  cyclonedds  reliable      9.25 Hz, 10.8 MB/s     <- works
#   B  cyclonedds  best_effort   9.33 Hz, 11.0 MB/s     <- works
#   C  fastrtps    reliable      matched NOTHING
#   D  fastrtps    best_effort   matched NOTHING
#
# Humble's Fast-DDS 2.6 will not match this robot's Foxy Fast-DDS 2.1 lidar
# writer at all — zero samples, not degraded delivery — while the 300-byte /IMU
# was unaffected, which disguised it as a message-size problem. CycloneDDS 0.10
# reads the same publishers fine. Hence rmw_cyclonedds_cpp in Dockerfile.deploy
# and docker-compose.yml, and hence NO Foxy relay container: it is not needed,
# and CycloneDDS 0.7 (what Foxy ships) segfaults on this robot's domain 0 anyway.
#
# Re-run this after any RMW, QoS, firmware or ROS-version change. It is the
# fastest way to tell "the transport broke" from "the robot stopped publishing".
#
# 4 cells: {CycloneDDS, Fast-DDS} x {reliable, best_effort}. Cell D is the
# ORIGINAL FAILING CONFIGURATION, kept as the control.
#
# Self-contained: the subscriber is embedded below, so this is the only file you
# need on the robot. `ros2 topic hz` is NOT used — it cannot set reliability.
#
# Run ON THE GOS HOST from the dir holding docker-compose.yml + config/:
#     bash test_humble_ingest.sh
# =============================================================================
IMG=${IMG:-m20-deploy:humble}
TOPIC=${TOPIC:-/LIDAR/POINTS}
SECS=${SECS:-12}
OUT=${OUT:-/tmp/humble_ingest}
mkdir -p "$OUT"

SUBPY=$(cat <<'PY'
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu

topic, rel, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
cls = Imu if 'IMU' in topic.upper() else PointCloud2

class S(Node):
    def __init__(self):
        super().__init__('ingest_probe')
        q = QoSProfile(depth=5)
        q.history = HistoryPolicy.KEEP_LAST
        q.reliability = (ReliabilityPolicy.RELIABLE if rel == 'reliable'
                         else ReliabilityPolicy.BEST_EFFORT)
        self.n = 0
        self.b = 0
        # raw=True: never deserialize a multi-MB cloud in Python, it would
        # distort the very rate we are trying to measure.
        self.create_subscription(cls, topic, self.cb, q, raw=True)
        self.create_timer(secs, self.done)
    def cb(self, m):
        self.n += 1
        self.b += len(m)
    def done(self):
        print(f'AVGHZ={self.n/secs:.2f} MBPS={self.b/1e6/secs:.2f} N={self.n}',
              flush=True)
        raise SystemExit(0)

rclpy.init()
try:
    rclpy.spin(S())
except SystemExit:
    pass
PY
)

cell () {                      # $1 label  $2 rmw  $3 env-flags  $4 reliability
  local label="$1" rmw="$2" cfgenv="$3" rel="$4"
  local out hz mbps verdict
  out=$(docker run --rm --network=host --ipc=host \
        -e ROS_DOMAIN_ID=0 \
        -e RMW_IMPLEMENTATION="$rmw" \
        $cfgenv \
        -v "$PWD/config:/root/ros_ws/config:ro" \
        "$IMG" bash -lc "cat > /tmp/probe.py <<'ZZEOFZZ'
$SUBPY
ZZEOFZZ
timeout $((SECS+8)) python3 /tmp/probe.py '$TOPIC' '$rel' $SECS 2>&1" 2>&1)
  echo "$out" > "$OUT/$label.log"
  hz=$(grep -oE 'AVGHZ=[0-9.]+' <<<"$out" | tail -1 | cut -d= -f2)
  mbps=$(grep -oE 'MBPS=[0-9.]+' <<<"$out" | tail -1 | cut -d= -f2)
  if [[ -n "$hz" ]] && awk "BEGIN{exit !($hz >= 8.0)}"; then
    verdict="PASS   ${hz} Hz  ${mbps} MB/s"
  elif [[ -n "$hz" ]] && awk "BEGIN{exit !($hz > 0)}"; then
    verdict="LOSSY  ${hz} Hz  ${mbps} MB/s"
  elif grep -qiE 'Segmentation|core dumped' <<<"$out"; then
    verdict="CRASH  (segfault - Cyclone cannot talk to the vendor stack)"
  elif [[ -n "$hz" ]]; then
    verdict="NO DATA (subscription matched nothing)"
  else
    verdict="ERROR  $(grep -oiE 'Error.*|Traceback.*' <<<"$out" | head -1)"
  fi
  printf '  %-24s %-20s %-12s %s\n' "$label" "$rmw" "$rel" "$verdict"
}

CYC="-e CYCLONEDDS_URI=file:///root/ros_ws/config/cyclonedds_gos.xml"
FRT="-e FASTRTPS_DEFAULT_PROFILES_FILE=/root/ros_ws/config/fastdds_profile_gos.xml -e CYCLONEDDS_URI="

echo "Reading $TOPIC from the Humble container, ${SECS}s per cell (logs: $OUT)"
printf '  %-24s %-20s %-12s %s\n' "CELL" "RMW" "QoS" "RESULT"
cell "A-cyclone-reliable"    rmw_cyclonedds_cpp "$CYC" reliable
cell "B-cyclone-besteffort"  rmw_cyclonedds_cpp "$CYC" best_effort
cell "C-fastrtps-reliable"   rmw_fastrtps_cpp   "$FRT" reliable
cell "D-fastrtps-besteffort" rmw_fastrtps_cpp   "$FRT" best_effort

cat <<'EOF'

EXPECTED (this is the shipped configuration's baseline)
  A and B PASS at ~9.3 Hz / ~11 MB/s; C and D report NO DATA.
  The ~9.3 of a nominal 10.03 Hz is mostly the probe's own discovery latency
  inside its fixed measurement window, not steady-state loss.

IF THAT CHANGED
  A/B now fail too  -> the robot stopped publishing, or discovery is broken.
                       Check `ros2 topic hz /LIDAR/POINTS` on the HOST first;
                       if the host sees 10 Hz, suspect participant-index
                       exhaustion (see config/cyclonedds_gos.xml).
  A/B CRASH         -> a CycloneDDS regression against the vendor's Fast-DDS
                       discovery data. This is what Foxy's 0.7 does; if 0.10
                       starts doing it, the non-DDS fallback is a Unix-socket
                       hop: Foxy + Fast-DDS ingest -> raw CDR -> Humble.
  C or D now PASS   -> Fast-DDS interop got fixed (firmware or ROS update).
                       Nothing to do, but worth recording in DEPLOYMENT.md.
EOF
