#!/usr/bin/env bash
# Rebuild everything that is Python-version specific, for Isaac Sim 5.1 (Python 3.11):
#   1. control_msgs / add_on_msgs  -> src/semu.robotics.ros2_bridge/{bin,packages}   (tools/Dockerfile.msgbuild)
#   2. Cython-compiled _ros2_bridge.cpython-311-*.so                                (inside the Isaac Sim image)
#   3. packaged extension tree     -> exts/semu.robotics.ros2_bridge                 (package_extension.bash)
# Usage: bash tools/build.sh [ROS_DISTRO=jazzy|humble] [isaac-sim image, default nvcr.io/nvidia/isaac-sim:5.1.0]
# The committed bin/ and packages/ are built for the distro written in packages/ROS_DISTRO.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${1:-jazzy}"
IMG="${2:-nvcr.io/nvidia/isaac-sim:5.1.0}"
SRC="$ROOT/src/semu.robotics.ros2_bridge"

echo "=== 1/3 messages for Python 3.11 (ROS 2 $ROS_DISTRO) ==="
docker build -q -t semu-ros2-bridge-msgbuild:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO -f "$ROOT/tools/Dockerfile.msgbuild" "$ROOT"
rm -rf "$SRC/bin" "$SRC/semu/robotics/ros2_bridge/packages"
docker run --rm -v "$SRC:/dst" semu-ros2-bridge-msgbuild:$ROS_DISTRO bash -c \
  'cp -r /out/bin /dst/bin && mkdir -p /dst/semu/robotics/ros2_bridge && cp -r /out/packages /dst/semu/robotics/ros2_bridge/packages && chown -R '"$(id -u):$(id -g)"' /dst/bin /dst/semu/robotics/ros2_bridge/packages'

echo "=== 2/3 Cython build of ros2_bridge.py in $IMG ==="
# --entrypoint: the image's default entrypoint starts the streaming app, not a shell
docker run --rm -u root -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -v "$ROOT:/work" -w /work/src/semu.robotics.ros2_bridge --entrypoint bash "$IMG" -c '
  set -e
  apt-get update -qq && apt-get install -y -qq --no-install-recommends build-essential > /dev/null
  /isaac-sim/python.sh -m pip install -q cython setuptools
  bash compile_extension.bash
  bash package_extension.bash
  chown -R '"$(id -u):$(id -g)"' /work/src /work/exts'

echo "=== 3/3 done ==="
ls -la "$SRC/semu/robotics/ros2_bridge/"*.so "$ROOT/exts/semu.robotics.ros2_bridge/semu/robotics/ros2_bridge/"*.so
