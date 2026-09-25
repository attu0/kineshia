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
"""

import sys
import time
import signal

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

        # Simple callback hooks the MainWindow can attach to for
        # feedback-driven sequencing (set by MainWindow after construction).
        self.on_status = None
        self.on_goal_reached = None

        # Time-series data buffers for plotting
        self.start_time = time.time()
        self.time_data = []
        self.q_data = [[], [], []]

        self.get_logger().info("GUI node started. Waiting for /joint_states...")

    def joint_callback(self, msg: JointState):
        """Processes incoming joint states and updates history."""
        if len(msg.position) >= 3:
            self.current_q = list(msg.position)[:3]

            # Record time series for PyQtGraph
            t = time.time() - self.start_time
            self.time_data.append(t)
            for i in range(3):
                self.q_data[i].append(self.current_q[i])

            # Maintain a rolling window of the last 200 data points
            if len(self.time_data) > 200:
                self.time_data.pop(0)
                for i in range(3):
                    self.q_data[i].pop(0)

    def status_callback(self, msg: String):
        """Handles /target_status -- surfaces reachability info from the controller."""
        text = msg.data
        self.last_status_is_warning = text.startswith("CLAMPED") or text.startswith("REJECTED") or text.startswith("IK_ERROR")
        if text.startswith("CLAMPED"):
            detail = text.split(" ", 1)[1] if " " in text else text
            self.last_status = "Target outside workspace -- clamped to boundary. " + detail
        elif text.startswith("REJECTED"):
            self.last_status = "Target rejected: solution violated joint limits / ground constraint."
        elif text.startswith("IK_ERROR"):
            self.last_status = f"IK error: {text}"
        else:
            self.last_status = "Target reachable, moving. " + text
        if self.on_status:
            self.on_status(self.last_status, self.last_status_is_warning)

    def goal_callback(self, msg: Bool):
        """Handles /at_goal -- lets the GUI know when a move has actually finished."""
        self.at_goal = msg.data
        if msg.data and self.on_goal_reached:
            self.on_goal_reached()

    def send_target(self, x, y):
        """Publishes the requested (x, y) target to the controller."""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.target_pub.publish(msg)
        self.get_logger().info(f"Published target: X={x:.2f}, Y={y:.2f}")


class MainWindow(QtWidgets.QWidget):
    """Main PyQt5 Window."""

    def __init__(self, ros_node: GuiNode):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("3-DOF Planar Arm Control Panel")
        self.resize(1920, 1080)

        self.init_ui()

        # SYNCHRONIZATION: Use QTimer to spin ROS 2 non-blockingly at ~33 Hz.
        # This weaves ROS callbacks seamlessly into the Qt event loop.
        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.timeout.connect(self.spin_and_update)
        self.spin_timer.start(30)

    def init_ui(self):
        # Main Layout
        layout = QtWidgets.QVBoxLayout(self)

        # ---- PLOTS SECTION ----
        plot_layout = QtWidgets.QHBoxLayout()

        # 1. Arm Visualization Plot
        self.arm_plot = pg.PlotWidget(title="Live Arm Visualization")
        self.arm_plot.setAspectLocked(True)
        self.arm_plot.setXRange(-7, 7)
        self.arm_plot.setYRange(-7, 7)
        self.arm_plot.showGrid(x=True, y=True)
        # Line representing the arm links
        self.arm_line = self.arm_plot.plot(pen=pg.mkPen('y', width=5), symbol='o', symbolBrush='b')
        plot_layout.addWidget(self.arm_plot)

        # 2. Joint Angles Plot
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

        # Telemetry Labels
        self.telemetry_label = QtWidgets.QLabel("Waiting for telemetry...")
        self.telemetry_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        control_layout.addWidget(self.telemetry_label)

        # Spacer
        control_layout.addStretch()

        # Target Inputs
        control_layout.addWidget(QtWidgets.QLabel("Target X:"))
        self.target_x_input = QtWidgets.QLineEdit("3.0")
        self.target_x_input.setFixedWidth(60)
        control_layout.addWidget(self.target_x_input)

        control_layout.addWidget(QtWidgets.QLabel("Target Y:"))
        self.target_y_input = QtWidgets.QLineEdit("2.0")
        self.target_y_input.setFixedWidth(60)
        control_layout.addWidget(self.target_y_input)

        # Send Button
        self.send_btn = QtWidgets.QPushButton("Send Target")
        self.send_btn.clicked.connect(self.on_send_clicked)
        control_layout.addWidget(self.send_btn)

        # Sequence Button
        self.seq_btn = QtWidgets.QPushButton("Run Test Sequence")
        self.seq_btn.clicked.connect(self.run_pick_and_place)
        self.seq_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        control_layout.addWidget(self.seq_btn)

        layout.addLayout(control_layout)

        # ---- STATUS BAR (surfaces /target_status, e.g. unreachable targets) ----
        self.status_label = QtWidgets.QLabel("No target sent yet.")
        self.status_label.setStyleSheet("font-size: 12px; padding: 4px;")
        layout.addWidget(self.status_label)

        # Wire the ROS node's callbacks to this window so status/goal
        # updates reach the GUI as soon as they're received, not just on
        # the next spin_and_update tick.
        self.node.on_status = self.on_target_status
        self.node.on_goal_reached = self.on_goal_reached

    def on_target_status(self, text, is_warning):
        color = "#b30000" if is_warning else "#1a7a1a"
        self.status_label.setStyleSheet(f"font-size: 12px; padding: 4px; color: {color}; font-weight: bold;")
        self.status_label.setText(text)

    def on_goal_reached(self):
        # Hook for feedback-driven sequencing; see run_pick_and_place().
        pass

    def spin_and_update(self):
        """Fired by QTimer: Spins ROS and updates the GUI."""
        # 1. Spin ROS 2 to process incoming /joint_states callbacks

        if not rclpy.ok():
            self.spin_timer.stop()
            return

        rclpy.spin_once(self.node, timeout_sec=0.0)

        # 2. Forward kinematics for plotting -- reuse the PROVIDED library
        #    (arm.forward_kinematics) instead of re-deriving the trig by
        #    hand, so the visualization can never drift from the arm's own
        #    definition of its geometry.
        q1, q2, q3 = self.node.current_q
        points = self.node.arm.forward_kinematics([q1, q2, q3])
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        x3, y3 = points[-1]

        self.node.current_ee = (x3, y3)

        # 3. Update Plots
        # Arm visualizer
        self.arm_line.setData(xs, ys)

        # Joint time-series
        if len(self.node.time_data) > 0:
            for i in range(3):
                self.q_lines[i].setData(self.node.time_data, self.node.q_data[i])

        # 4. Update Telemetry Text
        telemetry_text = (
            f"End Effector (x,y): ({x3:.2f}, {y3:.2f})  |  "
            f"Joints: q1={q1:.2f}, q2={q2:.2f}, q3={q3:.2f}"
        )
        self.telemetry_label.setText(telemetry_text)

    def on_send_clicked(self):
        """Reads UI inputs and commands the node to publish."""
        try:
            x = float(self.target_x_input.text())
            y = float(self.target_y_input.text())
            self.node.send_target(x, y)
        except ValueError:
            self.node.get_logger().error("Invalid Target Input. Please enter floats.")

    def run_pick_and_place(self):
        """Orchestrates the move -> pick -> move -> place sequence.

        Driven by the controller's /at_goal feedback rather than a guessed
        QTimer delay, so the sequence stays correct even if
        `trajectory_duration` changes, and won't fire the next step early
        (or too late) if a move takes longer than expected.
        """
        self.node.get_logger().info("Starting Pick-and-Place Sequence...")
        self.seq_btn.setEnabled(False)

        pick_target = (4.0, 2.0)
        place_target = (-3.0, 3.0)
        action_time_ms = 1000  # simulated gripper actuation, not arm motion

        # A tiny state machine over the goal-reached callback. Each step
        # waits for one /at_goal=True event before advancing.
        state = {"step": 0}

        def advance():
            step = state["step"]
            if step == 0:
                self.node.get_logger().info(f"Moving to pick target {pick_target}...")
                self.node.send_target(*pick_target)
            elif step == 1:
                self.node.get_logger().info("At pick target. Closing gripper (Pick)...")
                QtCore.QTimer.singleShot(action_time_ms, advance)
                state["step"] += 1
                return
            elif step == 2:
                self.node.get_logger().info(f"Moving to place target {place_target}...")
                self.node.send_target(*place_target)
            elif step == 3:
                self.node.get_logger().info("At place target. Opening gripper (Place)...")
                QtCore.QTimer.singleShot(action_time_ms, advance)
                state["step"] += 1
                return
            elif step == 4:
                self.node.get_logger().info("Sequence complete.")
                self.node.on_goal_reached = self.on_goal_reached  # detach sequence hook
                self.seq_btn.setEnabled(True)
                return
            state["step"] += 1

        def on_goal_during_sequence():
            # Only steps 0 and 2 are real arm moves that end in an /at_goal
            # event; steps 1/3 are timed gripper actions handled above.
            if state["step"] in (1, 3):
                advance()

        self.node.on_goal_reached = on_goal_during_sequence
        advance()


def main(args=None):
    rclpy.init(args=args)

    # 1. Create the QApplication
    app = QtWidgets.QApplication(sys.argv)

    # Safely tell Qt to quit its event loop on Ctrl+C
    signal.signal(signal.SIGINT, lambda sig, frame: app.quit())

    # A dummy timer that allows Python's signal handler to execute
    catch_timer = QtCore.QTimer()
    catch_timer.start(100)
    catch_timer.timeout.connect(lambda: None)

    # 2. Create the ROS 2 node instance
    node = GuiNode()

    # 3. Create and show the Qt GUI, passing the node inside
    window = MainWindow(node)
    window.show()

    # 4. Hand off execution to Qt's event loop
    try:
        app.exec_()  # Wait for app.quit() to be called
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()