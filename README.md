## semu.robotics.ros2_bridge — Isaac Sim 5.1 / ROS 2 Humble port

Fork of [Toni-SM/semu.robotics.ros2_bridge](https://github.com/Toni-SM/semu.robotics.ros2_bridge)
(MIT License). Branches `isaac5.1-humble` and `isaac5.1-jazzy` are the same extension rebuilt
for Isaac Sim 5.1 / Python 3.11 with the bundled ROS 2 Humble or Jazzy rclpy: the **Attribute** services (`add_on_msgs`), the
**FollowJointTrajectory** and **GripperCommand** action servers and a
`joint_trajectory` topic interface for an Isaac Sim articulation. It can be used either
as a Kit extension (`exts/semu.robotics.ros2_bridge`) or as a plain Python module imported
by the host application (this is how
[hsr-omniverse2](https://github.com/ry0hei-kobayashi/hsr-omniverse2) uses it, as a git
submodule).

What changed relative to upstream (details in
[docs/CHANGELOG.md](src/semu.robotics.ros2_bridge/docs/CHANGELOG.md)):

- All Python-3.7 / Foxy artifacts were rebuilt for Python 3.11 / Humble:
  `bin/` and `packages/` (`control_msgs`, `add_on_msgs`), the Cython-compiled
  `_ros2_bridge.cpython-311-*.so` and the packaged `exts/` tree. `tools/build.sh`
  regenerates all of them with Docker (see below). As upstream did, the built
  artifacts are committed.
- `add_on_msgs` is now also available as **source** (`src/add_on_msgs`, reconstructed
  from the upstream README / generated code; upstream never committed it).
- Components are no longer discovered through `semu.usd.schemas` prims. In extension
  mode they are listed in the setting `/exts/semu.robotics.ros2_bridge/components`
  (JSON, see `config/extension.toml`); in module mode the host constructs them.
- `physics_step()` / `update_step()` merged into `step(dt)`; topic-mode trajectories,
  controller state publishers and trajectory validation were added.

Upstream targets Isaac Sim 2022 (Python 3.7, Foxy message binaries, USD-schema
driven configuration). None of that works on Isaac Sim 5.1 (Python 3.11, bundled
ROS 2 Humble), hence this port. See [docs/CHANGELOG.md](src/semu.robotics.ros2_bridge/docs/CHANGELOG.md)
for the exact list of changes relative to upstream.

### Building (Python 3.11 artifacts)

```bash
bash tools/build.sh jazzy      # or humble; needs Docker, uses nvcr.io/nvidia/isaac-sim:5.1.0
```

`packages/ROS_DISTRO` records the distro the committed artifacts were built for
(`isaac5.1-jazzy` branch: jazzy, `isaac5.1-humble` branch: humble).

1. `tools/Dockerfile.msgbuild` builds `control_msgs` (the distro branch) and `src/add_on_msgs`
   for Python 3.11 in a `ros:<distro>-ros-base` box and writes `bin/` + `packages/`
   (RUNPATH set to `$ORIGIN/../../bin`, so no `LD_LIBRARY_PATH` is needed).
2. The Isaac Sim image compiles `ros2_bridge.py` with Cython
   (`compile_extension.bash`) and packages `exts/` (`package_extension.bash`).

### Requirements

- Isaac Sim 4.5 or 5.1 (`isaacsim.core.utils`, `omni.isaac.dynamic_control` — the
  latter is deprecated since 4.5 but still shipped in `extsDeprecated/` in 5.1).
- ROS 2 Humble Python packages importable from Isaac's Python: `rclpy`,
  `trajectory_msgs`, `control_msgs`. On Isaac 5.1 use the bundled
  `/isaac-sim/exts/isaacsim.ros2.bridge/humble/rclpy`; `control_msgs` is not bundled
  and has to be built for Python 3.11 separately.

### Usage as a Kit extension

Add `exts/` to the extension search path and enable `semu.robotics.ros2_bridge`, e.g.
`SimulationApp({...}, extra_args=["--ext-folder", "<repo>/exts"])` followed by
`enable_extension("semu.robotics.ros2_bridge")`. When the timeline starts playing the
components listed in `/exts/semu.robotics.ros2_bridge/components` are created; the
default is the Attribute services (`/get_prims`, `/get_prim_attributes`,
`/get_prim_attribute`, `/set_prim_attribute`).

### Usage as a Python module

```python
import sys
sys.path.insert(0, '<path to this repo>/src/semu.robotics.ros2_bridge')
from semu.robotics.ros2_bridge.ros2_bridge import (
    RosControlFollowJointTrajectory, RosControllerGripperCommand)

from omni.isaac.dynamic_control import _dynamic_control
dci = _dynamic_control.acquire_dynamic_control_interface()

arm = RosControlFollowJointTrajectory(node, dci)        # node: rclpy.node.Node
arm.start('/hsrb', '/arm_trajectory_controller/follow_joint_trajectory')
...
def on_physics_step(dt):
    arm.step(dt)                                       # call every physics step
```

- `start(articulation_path, action_name)` creates the action server, a
  `<controller>/joint_trajectory` topic subscriber and `<controller>/state`
  publishers. `stop()` tears them down.
- `controlled_joints` (class attribute, optional) restricts the joints reported in
  the controller state message.
- Trajectories are interpolated linearly between points at every `step(dt)`.

### Differences from upstream (summary)

- No `semu.usd.schemas` / `semu.robotics.ros_bridge_ui` dependency. `Ros2Bridge`
  creates its components from the `components` setting; `RosAttribute`,
  `RosControlFollowJointTrajectory` and `RosControllerGripperCommand` are
  constructed with `(node, dci)` and started with explicit service/action names.
- `physics_step()` / `update_step()` merged into a single `step(dt)`.
- Added topic-mode trajectories (`/<controller>/joint_trajectory`), controller
  state publishing, zero `time_from_start` padding and goal preemption.
- `bin/` and `packages/` contain Humble `control_msgs` and `add_on_msgs` built for
  Python 3.11 (`tools/build.sh`) instead of the Foxy / Python 3.7 binaries.

### License

MIT, see [LICENSE](LICENSE). Copyright (c) 2021 Toni-SM for the original work.
