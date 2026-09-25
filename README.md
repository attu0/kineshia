# planar_arm_control

ROS 2 (Jazzy/Humble) control + PyQt5/PyQtGraph telemetry GUI for a 3-DoF
planar manipulator. Take-home submission for Kineshia Robotics.

## Build

```bash
# from your workspace root, e.g. ~/ros2_ws
cp -r planar_arm_control src/
pip install PyQt5 pyqtgraph --user   # not available via rosdep on all distros
colcon build --packages-select planar_arm_control
source install/setup.bash
```

## Run

```bash
ros2 launch planar_arm_control bringup.launch.py
```

This brings up `controller_node` and `gui_node` together. Launch arguments:

- `publish_rate_hz` (default `50.0`) — rate of `/joint_states`.
- `control_mode` (default `position`) — reserved for future control modes
  (see "What I'd do next").

From the GUI: enter a target X/Y and click **Send Target**, or click
**Run Test Sequence** to run the fixed pick-and-place scenario below.

## Test scenario (as specified in the brief)

- Pick at `(4.0, 2.0)`, place at `(-3.0, 3.0)` — run via the **Run Test
  Sequence** button.
- Edge case: `(7.0, 3.0)` is outside the workspace (max reach = 6.5). Type
  it into the Target X/Y fields and click **Send Target** — the status bar
  at the bottom of the GUI turns red and reports the requested target
  alongside the clamped point actually used.

## What I built

- **`controller_node`** — owns arm state, subscribes to `/target_pose`,
  runs the provided `PlanarArm.inverse_kinematics`, and streams a quintic
  (zero velocity/acceleration at both ends) joint-space trajectory on
  `/joint_states` at a fixed rate.
- **`gui_node`** — PyQt5/PyQtGraph client: live arm visualization, joint
  angle vs. time plot, EE telemetry, target input, and the pick-and-place
  sequence button. Runs `rclpy.spin_once(timeout_sec=0.0)` from a `QTimer`
  so the Qt event loop and the ROS 2 side share one thread without either
  blocking the other.
- **`/target_status`** (`std_msgs/String`) and **`/at_goal`**
  (`std_msgs/Bool`) — added on top of the suggested interface so the
  controller can tell the GUI *why* the arm is doing something (target
  reachable / clamped / rejected) and *when* a move has actually finished,
  rather than the GUI guessing off a timer.
- **`/joint_command`** vs **`/joint_states`** — kept as separate topics on
  purpose. See the design note for why.

## Known limitations / what I'd do next with more time

- Only position/trajectory control is implemented; `control_mode` is
  declared but not yet branched on. Next: a velocity mode and a PID
  tracking loop with a live error plot (see design note, stretch goals).
- The Jacobian IK fallback (`jacobian_ik` in the provided library) doesn't
  itself check joint limits or the ground constraint. I added a check in
  `controller_node` that rejects any solution — analytical or
  fallback — that fails those constraints, and reports it on
  `/target_status`, rather than modifying `planar_arm.py`.
- No automated tests yet (`launch_testing` / `pytest`) — would add
  coverage for the reachability-clamp status logic and the
  constraint-rejection path first, since those are the two behaviors this
  submission adds beyond the stub.
- Single fixed test scenario is hardcoded in the GUI's sequence button;
  a real operator UI would let you queue arbitrary pick/place pairs.