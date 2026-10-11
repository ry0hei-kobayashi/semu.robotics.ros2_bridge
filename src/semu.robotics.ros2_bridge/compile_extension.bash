#!/bin/bash
# Compile ros2_bridge.py with Cython using Isaac Sim's own Python (5.1: Python 3.11).
# Run inside the Isaac Sim container (see ../../tools/build.sh); ISAAC_PYTHON can
# override the interpreter.
set -e
ISAAC_PYTHON="${ISAAC_PYTHON:-/isaac-sim/kit/python/bin/python3}"
export LIBRARY_PATH="$(dirname "$ISAAC_PYTHON")/../include"

# delete old files (the clean script removes files that may not exist yet)
set +e
. clean_extension.bash
set -e

# compile code
"$ISAAC_PYTHON" compile_extension.py build_ext --inplace

# move compiled file
mv *.so semu/robotics/ros2_bridge/

# delete temporal data
rm -r build
rm -f semu/robotics/ros2_bridge/*.c
