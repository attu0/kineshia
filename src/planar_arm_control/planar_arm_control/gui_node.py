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
import math
import time
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

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

        # State Data for GUI
        self.current_q = [0.0, 0.0, 0.0]
        self.current_ee = (0.0, 0.0)

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
        self.resize(1000, 600)

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

    def spin_and_update(self):
        """Fired by QTimer: Spins ROS and updates the GUI."""
        # 1. Spin ROS 2 to process incoming /joint_states callbacks

        if not rclpy.ok():
            self.spin_timer.stop()
            return

        rclpy.spin_once(self.node, timeout_sec=0.0)

        # 2. Compute Forward Kinematics for plotting
        q1, q2, q3 = self.node.current_q
        L1, L2, L3 = LINK_LENGTHS

        x0, y0 = 0.0, 0.0
        x1 = L1 * math.cos(q1)
        y1 = L1 * math.sin(q1)
        x2 = x1 + L2 * math.cos(q1 + q2)
        y2 = y1 + L2 * math.sin(q1 + q2)
        x3 = x2 + L3 * math.cos(q1 + q2 + q3)
        y3 = y2 + L3 * math.sin(q1 + q2 + q3)

        self.node.current_ee = (x3, y3)

        # 3. Update Plots
        # Arm visualizer
        self.arm_line.setData([x0, x1, x2, x3], [y0, y1, y2, y3])

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
        """Orchestrates the move -> pick -> move -> place sequence."""
        self.node.get_logger().info("Starting Pick-and-Place Sequence...")
        self.seq_btn.setEnabled(False)

        # Define the fixed test scenario
        pick_target = (4.0, 2.0)
        place_target = (-3.0, 3.0)

        # Timing (depends on your trajectory_duration parameter, assuming 2.0s)
        move_time = 2500  # ms
        action_time = 1000  # ms (simulated gripper action)

        # Step 1: Move to Pick
        self.node.send_target(*pick_target)

        # Step 2: "Pick" (Simulated delay, then move to Place)
        QtCore.QTimer.singleShot(
            move_time, lambda: self.node.get_logger().info("Closing gripper (Pick)..."))

        # Step 3: Move to Place
        QtCore.QTimer.singleShot(
            move_time + action_time,
            lambda: self.node.send_target(
                *place_target))

        # Step 4: "Place" & Reset Button
        def finish_sequence():
            self.node.get_logger().info("Opening gripper (Place). Sequence Complete.")
            self.seq_btn.setEnabled(True)

        QtCore.QTimer.singleShot((move_time * 2) + action_time, finish_sequence)


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
