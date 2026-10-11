#!/usr/bin/env bash
# Runs inside tools/Dockerfile.msgbuild. Builds control_msgs and add_on_msgs for
# Python 3.11 and lays the result out like the upstream extension expects:
#   /out/bin/        lib<pkg>__rosidl_*.so  (C typesupport, Python-version independent)
#   /out/packages/   <pkg>/  (Python package incl. *_s__rosidl_typesupport_*.cpython-311*.so)
# RUNPATH of every Python extension is rewritten to $ORIGIN/../../bin so that the
# packages work without LD_LIBRARY_PATH when placed under src/semu.robotics.ros2_bridge/.
set -euo pipefail
OUT=/out
PY=/py311/bin/python3.11

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
set +u; source /opt/ros/$ROS_DISTRO/setup.bash; set -u
# the generators (rosidl_*) live in the distro's own Python (humble: 3.10, jazzy: 3.12)
export PYTHONPATH=$(ls -d /opt/ros/$ROS_DISTRO/lib/python3.*/site-packages | head -1):${PYTHONPATH:-}

cd /ws
# Jazzy's rosidl calls FindPython3 several times; EXECUTABLE alone lets it pick the system
# 3.12, so pin the venv with ROOT_DIR / FIND_STRATEGY / FIND_VIRTUALENV and put it first in PATH.
PATH=/py311/bin:$PATH colcon build --merge-install --packages-select control_msgs add_on_msgs \
    --cmake-args -DPython3_EXECUTABLE=$PY -DPYTHON_EXECUTABLE=$PY \
                 -DPython3_ROOT_DIR=/py311 -DPython3_FIND_STRATEGY=LOCATION -DPython3_FIND_VIRTUALENV=ONLY \
                 -DCMAKE_BUILD_TYPE=Release

# Humble names the Python extensions *.cpython-310-*.so; Jazzy installs untagged
# <pkg>_s__rosidl_typesupport_*.so under lib/python3.11/ (and 3.12/). Check that the 3.11
# site-packages has extensions and that nothing links another libpython.
SP311=/ws/install/lib/python3.11/site-packages
n311=$(find "$SP311" -name '*.so' 2>/dev/null | wc -l)
bad=$(for so in $(find /ws/install -name '*.so'); do readelf -d "$so" 2>/dev/null | grep -qE 'libpython3\.(10|12)' && echo "$so"; done || true)
echo "python3.11 extensions: $n311"
[ "$n311" -gt 0 ] || { echo "FAIL: no Python 3.11 extension modules were built"; find /ws/install -name '*.so' | head; exit 1; }
[ -z "$bad" ] || { echo "FAIL: extensions linked against another libpython:"; echo "$bad"; exit 1; }

mkdir -p "$OUT/bin" "$OUT/packages"
for pkg in control_msgs add_on_msgs; do
    cp -f /ws/install/lib/lib${pkg}__*.so "$OUT/bin/"
    d=$(find "$SP311" -maxdepth 1 -type d -name "$pkg" | head -1)
    cp -rf "$d" "$OUT/packages/"
done
echo "$ROS_DISTRO" > "$OUT/packages/ROS_DISTRO"
find "$OUT/packages" -name __pycache__ -type d -prune -exec rm -rf {} +
# make the Python extensions find bin/ relative to themselves. In the extension tree the
# layout is <ext>/semu/robotics/ros2_bridge/packages/<pkg>/*.so and <ext>/bin/*.so, so bin/
# is five levels up from the extension module.
for so in "$OUT"/packages/*/*.so; do patchelf --set-rpath '$ORIGIN/../../../../../bin' "$so"; done
for so in "$OUT"/bin/*.so; do patchelf --set-rpath '$ORIGIN' "$so"; done
echo "bin/: $(ls "$OUT/bin" | wc -l) files, packages/: $(ls "$OUT/packages")"
