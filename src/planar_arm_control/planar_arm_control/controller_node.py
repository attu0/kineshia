#!/usr/bin/env python3
"""
controller_node.py

A ROS 2 node that owns the arm state, accepts a target, plans a
time-parameterized joint trajectory, and streams joint states as it executes.

Features:
    - Publishes:   /joint_states (sensor_msgs/JointState) at a fixed rate.
    - Subscribes:  /target_pose (geometry_msgs/Point).
    - Trajectory:  Quintic polynomial for smooth zero-velocity/acceleration starts and stops.
    - Uses the provided `PlanarArm` kinematics library.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# The provided kinematics library
from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")

        # Initialize kinematics
        self.arm = PlanarArm(LINK_LENGTHS)

        # Declare parameters
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_duration", 2.0)

        self.publish_rate = self.get_parameter("publish_rate_hz").value
        self.duration = self.get_parameter("trajectory_duration").value

        # Arm State
        self.current_q = [0.0, 0.0, 0.0]
        self.start_q = [0.0, 0.0, 0.0]
        self.target_q = [0.0, 0.0, 0.0]

        # Trajectory state
        self.is_moving = False
        self.move_start_time = 0.0

        # ROS 2 Interfaces
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_sub = self.create_subscription(Point, '/target_pose', self.target_callback, 10)

        # Control Loop Timer[cite: 2]
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(f"Controller started at {self.publish_rate} Hz.")

    def target_callback(self, msg: Point):
        """Triggered when a new target (X, Y) is received."""
        target_xy = (msg.x, msg.y)
        self.get_logger().info(f"Received target: {target_xy}")

        # 1. Run IK on the incoming target[cite: 2]
        try:
            solution = self.arm.inverse_kinematics(target_xy, initial_guess=self.current_q)

            # 2. Setup trajectory interpolation variables
            self.start_q = list(self.current_q)
            self.target_q = solution
            self.move_start_time = time.time()
            self.is_moving = True

            self.get_logger().info(f"IK Solution found. Moving joints to: "
                                   f"[{solution[0]:.2f}, {solution[1]:.2f}, {solution[2]:.2f}]")
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")

    def control_loop(self):
        """Steps along the trajectory and publishes JointState[cite: 2]."""

        if self.is_moving:
            elapsed = time.time() - self.move_start_time

            # Normalize time (t = 0.0 to 1.0)
            t = elapsed / self.duration

            if t >= 1.0:
                t = 1.0
                self.is_moving = False
                self.get_logger().info("Target reached.")

            # Generate a smooth joint-space trajectory (Quintic Polynomial)[cite: 2]
            # s(t) = 10t^3 - 15t^4 + 6t^5 ensures zero velocity and acceleration at ends.
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            for i in range(3):
                self.current_q[i] = self.start_q[i] + s * (self.target_q[i] - self.start_q[i])

        # Publish current state
        self.publish_state()

    def publish_state(self):
        """Constructs and publishes the JointState message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.current_q
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()