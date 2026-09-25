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
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool

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
        self.current_q = [0.0, 0.0, 0.0]   # commanded (planner) state
        self.measured_q = [0.0, 0.0, 0.0]  # reported (plant) state -- see simulate_plant()
        self.start_q = [0.0, 0.0, 0.0]
        self.target_q = [0.0, 0.0, 0.0]

        # Trajectory state
        self.is_moving = False
        self.move_start_time = 0.0

        # ROS 2 Interfaces
        # NOTE on the command/state seam: `/joint_command` is what the planner
        # WANTS the joints to do at each tick. `/joint_states` is what the
        # arm actually reports. In this simulation the "plant" is trivial
        # (state = command, see `simulate_plant()` below), so the two are
        # numerically identical today -- but keeping them as separate topics
        # means a real driver node (e.g. a Dynamixel interface publishing
        # true encoder feedback on /joint_states) can be dropped in later
        # without touching the planner at all.
        self.command_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/target_status', 10)
        self.goal_pub = self.create_publisher(Bool, '/at_goal', 10)
        self.target_sub = self.create_subscription(Point, '/target_pose', self.target_callback, 10)

        # Publish an initial "at goal" so GUI-side sequencing has a sane
        # starting value before the first target arrives.
        self.goal_pub.publish(Bool(data=True))

        # Control Loop Timer[cite: 2]
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(f"Controller started at {self.publish_rate} Hz.")

    def target_callback(self, msg: Point):
        """Triggered when a new target (X, Y) is received."""
        target_xy = (msg.x, msg.y)
        self.get_logger().info(f"Received target: {target_xy}")

        # 0. Work out (ourselves, without modifying planar_arm.py) whether
        #    the requested target lies outside the workspace, so we can
        #    tell the GUI *why* the arm is heading somewhere unexpected.
        #    reachable_target() is part of the library's public API, so
        #    calling it here is fine -- we're just not editing the file.
        max_reach = sum(self.arm.link_lengths)
        requested = np.asarray(target_xy, dtype=float)
        clamped_target = self.arm.reachable_target(target_xy)
        was_clamped = np.linalg.norm(requested) > max_reach

        # 1. Run IK on the incoming target
        try:
            solution = self.arm.inverse_kinematics(target_xy, initial_guess=self.current_q)
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            self.status_pub.publish(String(data=f"IK_ERROR:{e}"))
            return

        # 2. Validate the solution ourselves before committing to it.
        #    inverse_kinematics() already filters analytical candidates
        #    through within_joint_limits()/arm_above_base(), but its
        #    Jacobian fallback (jacobian_ik) does NOT -- so a solution
        #    reaching us here is not guaranteed to be constraint-safe.
        #    We refuse to stream an unsafe trajectory to /joint_states.
        solution = list(solution)
        if not self.arm.within_joint_limits(solution) or not self.arm.arm_above_base(solution):
            self.get_logger().error(
                f"Rejected IK solution for {target_xy}: violates joint limits "
                f"or ground constraint. Holding current position."
            )
            self.status_pub.publish(String(data="REJECTED:constraint_violation"))
            return

        # 3. Setup trajectory interpolation variables
        self.start_q = list(self.current_q)
        self.target_q = solution
        self.move_start_time = time.time()
        self.is_moving = True
        self.goal_pub.publish(Bool(data=False))

        # 4. Tell the GUI whether the target was reachable as-is or had to
        #    be clamped to the workspace boundary -- this is what
        #    surfaces the (7.0, 3.0) edge case sensibly instead of just
        #    printing to the controller's own stdout.
        if was_clamped:
            status = (
                f"CLAMPED:requested=({target_xy[0]:.2f},{target_xy[1]:.2f}) "
                f"used=({clamped_target[0]:.2f},{clamped_target[1]:.2f})"
            )
            self.get_logger().warn(status)
        else:
            status = f"OK:target=({target_xy[0]:.2f},{target_xy[1]:.2f})"
        self.status_pub.publish(String(data=status))

        self.get_logger().info(f"IK Solution found. Moving joints to: "
                               f"[{solution[0]:.2f}, {solution[1]:.2f}, {solution[2]:.2f}]")

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
                self.goal_pub.publish(Bool(data=True))

            # Generate a smooth joint-space trajectory (Quintic Polynomial)
            # s(t) = 10t^3 - 15t^4 + 6t^5 ensures zero velocity and acceleration at ends.
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            for i in range(3):
                self.current_q[i] = self.start_q[i] + s * (self.target_q[i] - self.start_q[i])

        # Publish the commanded joint values, then run them through the
        # (currently trivial) plant to get the reported /joint_states.
        self.publish_command()
        self.simulate_plant()
        self.publish_state()

    def publish_command(self):
        """What the planner wants the joints to be doing right now."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.current_q
        self.command_pub.publish(msg)

    def simulate_plant(self):
        """Stand-in for real actuator/encoder dynamics.

        In simulation the "plant" is an identity map: the reported state is
        exactly the commanded state. This is the seam where a hardware
        backend would plug in instead -- e.g. a Dynamixel driver node that
        publishes /joint_states from real encoder reads (with its own
        latency, noise, and possible tracking error against /joint_command),
        while this controller keeps generating commands unchanged.
        """
        self.measured_q = list(self.current_q)

    def publish_state(self):
        """Constructs and publishes the (measured) JointState message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.measured_q
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