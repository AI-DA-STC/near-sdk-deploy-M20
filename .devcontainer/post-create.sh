#!/usr/bin/env bash
# Runs once when the dev container is created (see devcontainer.json).
# NOTE: runs as root inside the container, so no sudo needed.
set -uo pipefail

WS=/root/ros_ws
SWAGGER_DIR="$WS/src/src/SWAGGER"

echo "==> Refreshing apt lists (the image wipes them to stay small)"
apt-get update

echo "==> Installing ROS dependencies for packages under src/"
rosdep update
rosdep install --from-paths "$WS/src" --ignore-src -r -y \
  --skip-keys "glim_ros cmake_modules" || true

echo "==> Installing navigation Python deps into system Python"
python3 -m pip install -r "$WS/requirements.txt"

if [ -d "$SWAGGER_DIR" ]; then
  echo "==> Setting up SWAGGER venv (offline graph tool)"
  python3 -m venv /root/swagger-venv
  # Long timeout + retries: the CUDA wheels are large and the mirror is flaky.
  /root/swagger-venv/bin/pip install --upgrade --default-timeout=1000 --retries 5 pip
  /root/swagger-venv/bin/pip install --default-timeout=1000 --retries 5 pyyaml
  /root/swagger-venv/bin/pip install --default-timeout=1000 --retries 5 -e "$SWAGGER_DIR"
  grep -q 'alias swagger-env' /root/.bashrc || \
    echo "alias swagger-env='source /root/swagger-venv/bin/activate'" >> /root/.bashrc
else
  echo "==> SWAGGER not found at $SWAGGER_DIR (submodule not initialised?), skipping"
fi

echo "==> post-create complete"
