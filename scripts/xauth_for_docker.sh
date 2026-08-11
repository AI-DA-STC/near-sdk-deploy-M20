#!/usr/bin/env bash
# =============================================================================
# xauth_for_docker.sh — build the X authority cookie the `rviz` compose service
# needs, then print how to launch it.
#
# Usage on the robot, inside an `ssh -Y` session:
#   ./scripts/xauth_for_docker.sh && docker compose run --rm rviz
#
# WHY THIS EXISTS
# `ssh -Y` sets DISPLAY=localhost:10.0 and writes a MIT-MAGIC-COOKIE into the
# login user's ~/.Xauthority, keyed to THIS host's name. A container is a
# different hostname as far as xauth is concerned, so handing it the file
# verbatim fails auth even though the socket connects. The fix is to rewrite the
# entry's address family to the wildcard 'ffff', which matches any host.
#
# The container reaches the tunnel over TCP (127.0.0.1:6010 for display :10),
# which works only because the rviz service is `network_mode: host` and so
# shares the robot's loopback. No /tmp/.X11-unix socket is involved for ssh -Y.
# =============================================================================
set -euo pipefail

XAUTH=/tmp/.docker.xauth

if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: DISPLAY is empty — this shell has no X forwarding." >&2
    echo "  * Reconnect with 'ssh -Y' (or -X)." >&2
    echo "  * If you then ran 'su'/'sudo -i', DISPLAY and XAUTHORITY were" >&2
    echo "    discarded: X forwarding belongs to the LOGIN user's session." >&2
    echo "    Re-export both, or use 'sudo -E' to preserve them." >&2
    echo "  * If DISPLAY is empty right after logging in, the robot is probably" >&2
    echo "    missing xauth (sshd disables forwarding silently without it):" >&2
    echo "        command -v xauth || sudo apt-get install -y xauth" >&2
    exit 1
fi

if ! command -v xauth >/dev/null; then
    echo "ERROR: xauth not installed on this host — install it and reconnect:" >&2
    echo "    sudo apt-get install -y xauth" >&2
    exit 1
fi

# A leftover DIRECTORY here is the classic failure: if the bind-mount source
# does not exist, Docker creates it as an empty dir and RViz then fails auth
# with a misleading error. Force it to be a plain file.
if [ -d "$XAUTH" ]; then
    echo ">>> removing stale directory $XAUTH (Docker created it as a dir)"
    rmdir "$XAUTH"
fi
rm -f "$XAUTH"
touch "$XAUTH"

# 'nlist' prints numeric entries; s/^..../ffff/ rewrites the address family to
# the wildcard so the cookie is valid from inside the container too.
if ! xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -; then
    echo "ERROR: no xauth entry for DISPLAY=$DISPLAY" >&2
    echo "  'xauth list' shows what this session actually holds." >&2
    exit 1
fi
chmod 644 "$XAUTH"

echo ">>> wrote $XAUTH for DISPLAY=$DISPLAY"
xauth -f "$XAUTH" list
echo
echo ">>> now run:  docker compose run --rm rviz"
