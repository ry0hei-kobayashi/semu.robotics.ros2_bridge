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

set +u; source /opt/ros/humble/setup.bash; set -u
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH:-}

cd /ws
colcon build --merge-install --packages-select control_msgs add_on_msgs \
    --cmake-args -DPython3_EXECUTABLE=$PY -DPYTHON_EXECUTABLE=$PY -DCMAKE_BUILD_TYPE=Release

n311=$(find /ws/install -name '*cpython-311*.so' | wc -l)
n310=$(find /ws/install -name '*cpython-310*.so' | wc -l)
echo "cpython-311: $n311 / cpython-310: $n310"
[ "$n311" -gt 0 ] && [ "$n310" -eq 0 ] || { echo "FAIL: wrong Python ABI in build output"; exit 1; }

mkdir -p "$OUT/bin" "$OUT/packages"
for pkg in control_msgs add_on_msgs; do
    cp -f /ws/install/lib/lib${pkg}__*.so "$OUT/bin/"
    d=$(find /ws/install -type d -name "$pkg" -path '*site-packages*' | head -1)
    cp -rf "$d" "$OUT/packages/"
done
find "$OUT/packages" -name __pycache__ -type d -prune -exec rm -rf {} +
# make the Python extensions find bin/ relative to themselves. In the extension tree the
# layout is <ext>/semu/robotics/ros2_bridge/packages/<pkg>/*.so and <ext>/bin/*.so, so bin/
# is five levels up from the extension module.
for so in "$OUT"/packages/*/*.so; do patchelf --set-rpath '$ORIGIN/../../../../../bin' "$so"; done
for so in "$OUT"/bin/*.so; do patchelf --set-rpath '$ORIGIN' "$so"; done
echo "bin/: $(ls "$OUT/bin" | wc -l) files, packages/: $(ls "$OUT/packages")"
