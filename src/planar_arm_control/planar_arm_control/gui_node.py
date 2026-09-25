#!/usr/bin/env python3
"""
gui_node.py

A PyQt5 + PyQtGraph node that acts as a live client for a 3-DOF planar arm controller.
Features:
    - QTimer-based ROS 2 / Qt event loop integration (single-threaded, safe GUI updates).
    - Live subscription to /joint_states.
    - PyQtGraph plots for live arm visualization and joint angle telemetry over time.
    - User input fields to publish target X/Y coordinates.
    - Live end-effector (EE) telemetry.
    - Record and Playback engine for custom waypoint sequences (with loop limits).
"""

import sys
import time
import signal
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# Fallback stub for PlanarArm if the module isn't strictly available in this directory
try:
    from planar_arm_control.planar_arm import PlanarArm
except ImportError:
    class PlanarArm:
        def __init__(self, links):
            self.links = links

LINK_LENGTHS = [3.0, 2.0, 1.5]


class GuiNode(Node):
    """ROS 2 Node handling communications."""

    def __init__(self):
        super().__init__("gui_node")
        self.arm = PlanarArm(LINK_LENGTHS)

        # ROS 2 Interfaces
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.target_pub = self.create_publisher(Point, '/target_pose', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper_state', 10)
        self.status_sub = self.create_subscription(
            String, '/target_status', self.status_callback, 10
        )
        self.goal_sub = self.create_subscription(
            Bool, '/at_goal', self.goal_callback, 10
        )

        # State Data for GUI
        self.current_q = [0.0, 0.0, 0.0]
        self.current_ee = (0.0, 0.0)
        self.last_status = "No target sent yet."
        self.last_status_is_warning = False
        self.at_goal = True

        self.on_status = None
        self.on_goal_reached = None

        self.start_time = time.time()
        self.time_data = []
        self.q_data = [[], [], []]

        self.get_logger().info("[STARTUP] gui_node ready -- waiting for /joint_states...")

    def joint_callback(self, msg: JointState):
        if len(msg.position) >= 3:
            self.current_q = list(msg.position)[:3]
            t = time.time() - self.start_time
            self.time_data.append(t)
            for i in range(3):
                self.q_data[i].append(self.current_q[i])

            if len(self.time_data) > 200:
                self.time_data.pop(0)
                for i in range(3):
                    self.q_data[i].pop(0)

    def status_callback(self, msg: String):
        text = msg.data
        self.last_status_is_warning = text.startswith("CLAMPED") or text.startswith("REJECTED") or text.startswith("IK_ERROR")

        if text.startswith("CLAMPED"):
            detail = text.split(" ", 1)[1] if " " in text else text
            self.last_status = "Target outside workspace -- clamped to boundary. " + detail
            self.get_logger().warn(f"[WARNING] {text}")
        elif text.startswith("REJECTED"):
            self.last_status = "Target rejected: solution violated joint limits / ground constraint."
            self.get_logger().error(f"[FAILED] {text}")
        elif text.startswith("IK_ERROR"):
            self.last_status = f"IK error: {text}"
            self.get_logger().error(f"[FAILED] {text}")
        else:
            self.last_status = "Target reachable, moving. " + text
            self.get_logger().info(f"[RECEIVED] {text}")

        if self.on_status:
            self.on_status(self.last_status, self.last_status_is_warning)

    def goal_callback(self, msg: Bool):
        self.at_goal = msg.data
        if msg.data:
            self.get_logger().info("[REACHED] controller reports at_goal=True")
        if msg.data and self.on_goal_reached:
            self.on_goal_reached()

    def send_target(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.target_pub.publish(msg)
        self.get_logger().info(f"[SENT] target=({x:.2f}, {y:.2f})")

    def set_gripper(self, closed: bool):
        self.gripper_pub.publish(Bool(data=closed))
        self.get_logger().info(f"[GRIPPER] {'closing' if closed else 'opening'}")


class MainWindow(QtWidgets.QWidget):
    """Main PyQt5 Window."""

    def __init__(self, ros_node: GuiNode):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("3-DOF Planar Arm Control Panel")
        self.resize(1920, 1080)

        self.gripper_frac = 0.0
        self.gripper_target = 0.0

        # Loop and record features
        self.is_recording = False
        self.is_looping = False
        self.recorded_sequence = []
        self.playback_index = 0
        self.remaining_loops = 0

        self.init_ui()

        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.timeout.connect(self.spin_and_update)
        self.spin_timer.start(30)

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # ---- PLOTS SECTION ----
        plot_layout = QtWidgets.QHBoxLayout()

        self.arm_plot = pg.PlotWidget(title="Live Arm Visualization")
        self.arm_plot.setAspectLocked(True)
        self.arm_plot.setXRange(-7, 7)
        self.arm_plot.setYRange(-7, 7)
        self.arm_plot.showGrid(x=True, y=True)
        self.arm_line = self.arm_plot.plot(pen=pg.mkPen('y', width=5), symbol='o', symbolBrush='b')
        self.gripper_item = self.arm_plot.plot(pen=pg.mkPen('orange', width=4))
        plot_layout.addWidget(self.arm_plot)

        self.joint_plot = pg.PlotWidget(title="Joint Angles vs Time")
        self.joint_plot.setLabel('left', 'Angle (rad)')
        self.joint_plot.setLabel('bottom', 'Time (s)')
        self.joint_plot.addLegend()
        self.q_lines = [
            self.joint_plot.plot(pen='r', name="q1 (Base)"),
            self.joint_plot.plot(pen='g', name="q2 (Elbow)"),
            self.joint_plot.plot(pen='c', name="q3 (Wrist)")
        ]
        plot_layout.addWidget(self.joint_plot)
        layout.addLayout(plot_layout)

        # ---- CONTROLS & TELEMETRY SECTION ----
        control_layout = QtWidgets.QHBoxLayout()

        self.telemetry_label = QtWidgets.QLabel("Waiting for telemetry...")
        self.telemetry_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        control_layout.addWidget(self.telemetry_label)

        control_layout.addStretch()

        # Record / Playback Buttons
        self.record_btn = QtWidgets.QPushButton("Start Recording")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self.toggle_record)
        control_layout.addWidget(self.record_btn)

        # Loop count input
        control_layout.addWidget(QtWidgets.QLabel("Loops:"))
        self.loop_count_input = QtWidgets.QSpinBox()
        self.loop_count_input.setMinimum(1)
        self.loop_count_input.setMaximum(999)
        self.loop_count_input.setValue(1)
        self.loop_count_input.setFixedWidth(50)
        control_layout.addWidget(self.loop_count_input)

        self.loop_btn = QtWidgets.QPushButton("Play Sequence")
        self.loop_btn.setCheckable(True)
        self.loop_btn.clicked.connect(self.toggle_loop)
        control_layout.addWidget(self.loop_btn)

        self.clear_btn = QtWidgets.QPushButton("Clear Sequence")
        self.clear_btn.clicked.connect(self.clear_sequence)
        control_layout.addWidget(self.clear_btn)

        # Target Inputs
        control_layout.addWidget(QtWidgets.QLabel("Target X:"))
        self.target_x_input = QtWidgets.QLineEdit("3.0")
        self.target_x_input.setFixedWidth(60)
        control_layout.addWidget(self.target_x_input)

        control_layout.addWidget(QtWidgets.QLabel("Target Y:"))
        self.target_y_input = QtWidgets.QLineEdit("2.0")
        self.target_y_input.setFixedWidth(60)
        control_layout.addWidget(self.target_y_input)

        self.send_btn = QtWidgets.QPushButton("Send Target")
        self.send_btn.clicked.connect(self.on_send_clicked)
        control_layout.addWidget(self.send_btn)

        self.gripper_btn = QtWidgets.QPushButton("Close Gripper")
        self.gripper_btn.clicked.connect(self.on_gripper_toggle_clicked)
        control_layout.addWidget(self.gripper_btn)

        self.seq_btn = QtWidgets.QPushButton("Run Test Sequence")
        self.seq_btn.clicked.connect(self.run_pick_and_place)
        self.seq_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        control_layout.addWidget(self.seq_btn)

        layout.addLayout(control_layout)

        # ---- STATUS BAR ----
        self.status_label = QtWidgets.QLabel("No target sent yet.")
        self.status_label.setStyleSheet("font-size: 12px; padding: 4px;")
        layout.addWidget(self.status_label)

        self.node.on_status = self.on_target_status
        self.node.on_goal_reached = self.on_goal_reached

    def on_target_status(self, text, is_warning):
        color = "#b30000" if is_warning else "#1a7a1a"
        self.status_label.setStyleSheet(f"font-size: 12px; padding: 4px; color: {color}; font-weight: bold;")
        self.status_label.setText(text)

    def on_goal_reached(self):
        pass

    def spin_and_update(self):
        if not rclpy.ok():
            self.spin_timer.stop()
            return

        rclpy.spin_once(self.node, timeout_sec=0.0)

        q1, q2, q3 = self.node.current_q
        points = self.node.arm.forward_kinematics([q1, q2, q3])
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        x3, y3 = points[-1]

        self.node.current_ee = (x3, y3)
        self.arm_line.setData(xs, ys)

        self.gripper_frac += (self.gripper_target - self.gripper_frac) * 0.25
        heading = q1 + q2 + q3
        finger_len = 0.7
        max_spread = math.radians(35)
        spread = max_spread * (1.0 - self.gripper_frac)
        f1x = x3 + finger_len * math.cos(heading + spread)
        f1y = y3 + finger_len * math.sin(heading + spread)
        f2x = x3 + finger_len * math.cos(heading - spread)
        f2y = y3 + finger_len * math.sin(heading - spread)
        self.gripper_item.setData([f1x, x3, f2x], [f1y, y3, f2y])

        if len(self.node.time_data) > 0:
            for i in range(3):
                self.q_lines[i].setData(self.node.time_data, self.node.q_data[i])

        telemetry_text = (
            f"End Effector (x,y): ({x3:.2f}, {y3:.2f})  |  "
            f"Joints: q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f}"
        )
        self.telemetry_label.setText(telemetry_text)

    # ---- RECORD & PLAYBACK SYSTEM ----

    def toggle_record(self, checked):
        self.is_recording = checked
        self.record_btn.setText("Stop Recording" if checked else "Start Recording")
        if checked:
            self.node.get_logger().info("[RECORD] Started recording actions.")

    def clear_sequence(self):
        self.recorded_sequence.clear()
        self.node.get_logger().info("[RECORD] Sequence cleared.")

    def on_send_clicked(self):
        try:
            x = float(self.target_x_input.text())
            y = float(self.target_y_input.text())
            if self.is_recording:
                self.recorded_sequence.append({'type': 'move', 'target': (x, y)})
            self.node.send_target(x, y)
        except ValueError:
            self.node.get_logger().error("Invalid Target Input.")

    def open_gripper(self):
        self.gripper_target = 0.0
        if self.is_recording:
            self.recorded_sequence.append({'type': 'gripper', 'state': False})
        self.node.set_gripper(closed=False)
        self.gripper_btn.setText("Close Gripper")

    def close_gripper(self):
        self.gripper_target = 1.0
        if self.is_recording:
            self.recorded_sequence.append({'type': 'gripper', 'state': True})
        self.node.set_gripper(closed=True)
        self.gripper_btn.setText("Open Gripper")

    def on_gripper_toggle_clicked(self):
        if self.gripper_target >= 0.5:
            self.open_gripper()
        else:
            self.close_gripper()

    def toggle_loop(self, checked):
        self.is_looping = checked
        self.loop_btn.setText("Stop Sequence" if checked else "Play Sequence")

        if checked:
            if not self.recorded_sequence:
                self.node.get_logger().warn("[PLAYBACK] No sequence recorded!")
                self.loop_btn.setChecked(False)
                self.loop_btn.setText("Play Sequence")
                self.is_looping = False
                return

            self.playback_index = 0
            self.remaining_loops = self.loop_count_input.value()
            self.node.on_goal_reached = self.advance_playback
            self.node.get_logger().info(f"[PLAYBACK] Starting sequence for {self.remaining_loops} loop(s)...")

            # Disable input so user doesn't change loop count mid-run
            self.loop_count_input.setEnabled(False)
            self.advance_playback()
        else:
            self.node.get_logger().info("[PLAYBACK] Sequence stopped early.")
            self.loop_count_input.setEnabled(True)
            self.node.on_goal_reached = self.on_goal_reached

    def advance_playback(self):
        if not self.is_looping:
            return

        # Check if we reached the end of the sequence
        if self.playback_index >= len(self.recorded_sequence):
            self.remaining_loops -= 1
            if self.remaining_loops <= 0:
                # Finished all loops
                self.is_looping = False
                self.loop_btn.setChecked(False)
                self.loop_btn.setText("Play Sequence")
                self.loop_count_input.setEnabled(True)
                self.node.get_logger().info("[PLAYBACK] Sequence finished all loops.")
                self.node.on_goal_reached = self.on_goal_reached
                return
            else:
                self.node.get_logger().info(f"[PLAYBACK] Loop complete. {self.remaining_loops} loop(s) remaining.")
                self.playback_index = 0 # Reset for the next loop

        action = self.recorded_sequence[self.playback_index]
        self.playback_index += 1

        if action['type'] == 'move':
            tx, ty = action['target']
            self.node.get_logger().info(f"[PLAYBACK] Moving to ({tx}, {ty})")
            self.node.send_target(tx, ty)

        elif action['type'] == 'gripper':
            closing = action['state']
            self.node.get_logger().info(f"[PLAYBACK] Gripper -> {'Closed' if closing else 'Open'}")
            if closing:
                self.node.set_gripper(closed=True)
                self.gripper_target = 1.0
            else:
                self.node.set_gripper(closed=False)
                self.gripper_target = 0.0

            QtCore.QTimer.singleShot(1000, self.advance_playback)

    # ---- TEST SCENARIO ----

    def run_pick_and_place(self):
        self.node.get_logger().info("[SEQUENCE] starting pick-and-place test scenario")
        self.seq_btn.setEnabled(False)

        pick_target = (4.0, 2.0)
        place_target = (-3.0, 3.0)
        action_time_ms = 1000

        state = {"step": 0}

        def advance():
            step = state["step"]
            if step == 0:
                self.node.get_logger().info(f"[SEQUENCE] moving to pick target {pick_target}")
                self.node.send_target(*pick_target)
            elif step == 1:
                self.node.get_logger().info("[SEQUENCE] at pick target -- closing gripper")
                self.close_gripper()
                QtCore.QTimer.singleShot(action_time_ms, advance)
                state["step"] += 1
                return
            elif step == 2:
                self.node.get_logger().info(f"[SEQUENCE] moving to place target {place_target}")
                self.node.send_target(*place_target)
            elif step == 3:
                self.node.get_logger().info("[SEQUENCE] at place target -- opening gripper")
                self.open_gripper()
                QtCore.QTimer.singleShot(action_time_ms, advance)
                state["step"] += 1
                return
            elif step == 4:
                self.node.get_logger().info("[SEQUENCE] complete")
                self.node.on_goal_reached = self.on_goal_reached
                self.seq_btn.setEnabled(True)
                return
            state["step"] += 1

        def on_goal_during_sequence():
            if state["step"] in (1, 3):
                advance()

        self.node.on_goal_reached = on_goal_during_sequence
        advance()


def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda sig, frame: app.quit())

    catch_timer = QtCore.QTimer()
    catch_timer.start(100)
    catch_timer.timeout.connect(lambda: None)

    node = GuiNode()
    window = MainWindow(node)
    window.show()

    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()