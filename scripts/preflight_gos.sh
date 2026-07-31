#!/usr/bin/env bash
# =============================================================================
# preflight_gos.sh — run on the GOS host BEFORE `docker compose up`.
#
# Checks the one kernel setting that decides whether a 1.8 MB /LIDAR/POINTS
# scan can cross a UDP socket at all, and reports what the M20 is publishing.
#
#   ./scripts/preflight_gos.sh          # check only
#   sudo ./scripts/preflight_gos.sh -f  # check and fix (persists the sysctl)
#
# WHY: DDS asks the kernel for a big socket receive buffer; the kernel silently
# clamps that request to net.core.rmem_max. On a stock Ubuntu (212992 = 208 KB,
# accounted as ~416 KB usable) a multi-megabyte point cloud arrives as a burst of
# near-64 KB datagrams, overruns the buffer and loses fragments — and one lost
# fragment discards the ENTIRE sample, while a 300-byte Imu is never affected.
# Measured on a stock kernel, a 1.8 MB cloud managed ~1-2 Hz out of 10 over BOTH
# Fast-DDS and CycloneDDS.
#
# NOT the historical cause on THIS robot, which ships 562 MB. Kept as a guard:
# config/cyclonedds_gos.xml asks CycloneDDS for 8 MB as a hard requirement, so if
# this ever regresses the containers refuse to start rather than silently drop
# most scans. The actual cause of the lidar outage was the RMW — Humble's
# Fast-DDS matches NOTHING from this robot's Foxy Fast-DDS lidar writer, while
# CycloneDDS reads it fine. See DEPLOYMENT.md and scripts/test_humble_ingest.sh.
# =============================================================================
set -uo pipefail

WANT=16777216   # 16 MB — comfortably above one 1.8 MB scan in flight
FIX=0
[[ "${1:-}" == "-f" || "${1:-}" == "--fix" ]] && FIX=1

fail=0
note() { printf '  %s\n' "$*"; }

echo "== net.core.rmem_max =="
have=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
if (( have >= WANT )); then
  note "OK: $have (>= $WANT)"
else
  note "TOO SMALL: $have  (need >= $WANT)"
  note "A 1.8 MB PointCloud2 cannot be received reliably at this value, and"
  note "the CycloneDDS configs request 8 MB, so the containers will refuse to"
  note "start with: 'failed to increase socket receive buffer size'."
  if (( FIX )); then
    if sysctl -w net.core.rmem_max=$WANT >/dev/null 2>&1; then
      note "FIXED for this boot."
      # Persist. --network=host containers share the host's netns, and
      # net.core.* is global, so this cannot be set from inside a container:
      # it has to live on the host.
      printf 'net.core.rmem_max = %s\nnet.core.rmem_default = %s\n' \
        "$WANT" 1048576 > /etc/sysctl.d/60-m20-dds.conf \
        && note "Persisted to /etc/sysctl.d/60-m20-dds.conf"
    else
      note "FAILED to set — are you root? Try: sudo $0 -f"; fail=1
    fi
  else
    note "Fix with:  sudo $0 -f"
    note "Or by hand: sudo sysctl -w net.core.rmem_max=$WANT"
    fail=1
  fi
fi

echo
echo "== what the M20 is publishing (host Foxy/Fast-DDS side) =="
if command -v ros2 >/dev/null 2>&1; then
  # Deliberately NOT run inside a container: this is the host's own Foxy stack,
  # the one m20_bridge has to be able to see (over CycloneDDS).
  topics=$(timeout 10 ros2 topic list 2>/dev/null)
  for t in /LIDAR/POINTS /IMU; do
    if grep -qx "$t" <<<"$topics"; then
      note "OK: $t present"
      # reliability here is what m20_bridge's restamp_* must match; a mismatch
      # matches nothing and is indistinguishable from a transport fault.
      timeout 10 ros2 topic info -v "$t" 2>/dev/null \
        | grep -iE 'Type|Reliability' | sed 's/^/      /' | head -4
    else
      note "MISSING: $t — m20_bridge will have nothing to restamp"; fail=1
    fi
  done
else
  note "SKIP: no ros2 on PATH. Source the host's Foxy setup.bash and re-run,"
  note "or check by hand that /LIDAR/POINTS and /IMU are being published."
fi

echo
if (( fail )); then
  echo "PREFLIGHT FAILED — fix the above before 'docker compose up'."
else
  echo "PREFLIGHT OK."
fi
exit $fail
