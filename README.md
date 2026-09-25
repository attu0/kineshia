# planar_arm_control

ROS 2 (Jazzy) control + PyQt5/PyQtGraph telemetry GUI for a 3-DoF planar manipulator.

Take-home submission for Kineshia Robotics.

## 🛠️ Initial Setup

Clone the repository:

```bash
git clone https://github.com/attu0/kineshia.git
cd ~/kineshia
```

### 1. Install Required Packages and Libraries

The repository includes setup scripts under `src/planar_arm_control/scripts/`.

```bash
cd ~/kineshia/src/planar_arm_control/scripts

chmod +x *

./requirements.sh
```

The dependency script is configured for **Ubuntu 24.04 + ROS 2 Jazzy**.

### 2. Build the Package

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash

cd ~/kineshia
colcon build --symlink-install

source install/setup.bash
```

---

## 🚀 How to Run

The package includes `launch.sh` for easier startup.

```bash
cd ~/kineshia/src/planar_arm_control/scripts
```

### Recommended

```bash
./launch.sh
```

This starts the system in **velocity mode**.

### Position Mode

```bash
./launch.sh position
```

### Velocity Mode

```bash
./launch.sh velocity
```

You can also launch directly using ROS 2:

```bash
ros2 launch planar_arm_control bringup.launch.py     control_mode:=velocity     max_joint_velocity:=1.5
```

### Launch Arguments

- `publish_rate_hz` (default `50.0`) — Rate of `/joint_states` publication.
- `control_mode` (default `position`) — Selects the trajectory planner:
  - `position` — fixed-duration quintic blend
  - `velocity` — physics-based trapezoidal profile
- `max_joint_velocity` (default `1.5`) — Maximum velocity in rad/s for velocity mode.

---

## 🧪 What to Try

This package implements the core requirements along with two stretch goals.

### Test 1: Required Pick-and-Place

**Goal:** Verify the arm can execute a multi-step sequence with gripper actions.

1. Click the green **Run Test Sequence** button.
2. The arm should:
   - Move to `(4.0, 2.0)`
   - Close the gripper
   - Move to `(-3.0, 3.0)`
   - Open the gripper
3. Observe the PyQtGraph plots and gripper visualization.

### Test 2: Out-of-Bounds Target

**Goal:** Verify safe handling of unreachable targets.

1. Enter:
   - Target X: `7.0`
   - Target Y: `3.0`
2. Click **Send Target**.
3. The controller should clamp the target to the reachable boundary `(5.97, 2.56)`.
4. The GUI status bar should turn red and report that the target was clamped.

### Test 3: Velocity Control Mode

**Goal:** Verify the time-parameterized velocity profile.

1. Launch using:

```bash
ros2 launch planar_arm_control bringup.launch.py     control_mode:=velocity     max_joint_velocity:=1.5
```

2. Send a nearby target, for example `(3.0, 2.0)`.
3. Send a farther target, for example `(-3.0, 3.0)`.
4. Check the terminal output. The travel time should change according to the motion distance and velocity/acceleration limits.

### Test 4: Record & Playback

**Goal:** Verify custom sequence recording and playback.

1. Click **Start Recording**.
2. Send a target, e.g. `(2.0, 4.0)`.
3. Click **Close Gripper**.
4. Send another target, e.g. `(-2.0, 4.0)`.
5. Click **Open Gripper**.
6. Click **Stop Recording**.
7. Set **Loops** to `2`.
8. Click **Play Sequence**.

The recorded routine should execute twice and then stop automatically.

---

## 🏗️ Architecture Highlights

- **`controller_node`** — Owns the arm state, receives `/target_pose`, runs IK, validates joint/ground constraints, and generates the trajectory.
- **`gui_node`** — PyQt5/PyQtGraph interface for target control, visualization, telemetry, gripper control, and recording/playback.
- **Feedback-driven sequencing** — `/target_status` and `/at_goal` provide controller feedback so playback advances when the target is actually reached.
- **Sim-to-real seam** — `/joint_command` and `/joint_states` are kept separate. The current `simulate_plant()` mirrors the commanded state, allowing a real hardware driver to replace the simulated plant later.