#!/usr/bin/env python3

'''
*****************************************************************************************
*
*                    ===========================================
*                       planar_arm_control
*                    ===========================================
*
*  ROS 2 controller node for the 3-DOF planar arm.
*
*****************************************************************************************
'''

# Author:           Atharv Mudse
# Mail ID:          atharvmudse@gmail.com
# Filename:         controller_node.py
# Functions:        main, target_callback, control_loop, trajectory helpers
# Nodes:            controller_node
#
# Publishing Topics:
#                   /joint_command
#                   /joint_states
#                   /target_status
#                   /at_goal
#
# Subscribing Topics:
#                   /target_pose


################### IMPORT MODULES #######################
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool

# The provided kinematics library
from planar_arm_control.planar_arm import PlanarArm

##################### TASK CONSTANTS #######################

LINK_LENGTHS = [3.0, 2.0, 1.5]


##################### CLASS DEFINITION #######################

class ControllerNode(Node):
    '''
    Description: ROS 2 node that owns the arm state, accepts target
                 positions, plans trajectories, and publishes joint state
                 feedback.
    '''

    def __init__(self):
        super().__init__('controller_node')

        # Initialize kinematics
        self.arm = PlanarArm(LINK_LENGTHS)

        # Declare parameters
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_duration", 2.0)
        # Second control mode (stretch goal): "position" runs the original
        # fixed-duration quintic blend; "velocity" runs a trapezoidal
        # velocity-limited profile instead, capped at max_joint_velocity /
        # max_joint_acceleration rather than a preset duration.
        self.declare_parameter("control_mode", "position")
        self.declare_parameter("max_joint_velocity", 1.5)      # rad/s, velocity mode only
        self.declare_parameter("max_joint_acceleration", 2.0)  # rad/s^2, velocity mode only

        self.publish_rate = self.get_parameter("publish_rate_hz").value
        self.duration = self.get_parameter("trajectory_duration").value
        self.max_joint_velocity = self.get_parameter("max_joint_velocity").value
        self.max_joint_acceleration = self.get_parameter("max_joint_acceleration").value

        control_mode = self.get_parameter("control_mode").value
        if control_mode not in ("position", "velocity"):
            self.get_logger().warn(
                f"[WARNING] unknown control_mode '{control_mode}' -- falling back to 'position'"
            )
            control_mode = "position"
        self.control_mode = control_mode

        ##################### ARM STATE #######################
        self.current_q = [0.0, 0.0, 0.0]   # commanded (planner) state
        self.current_qdot = [0.0, 0.0, 0.0]  # commanded joint velocities (rad/s)
        self.measured_q = [0.0, 0.0, 0.0]  # reported (plant) state -- see simulate_plant()
        self.measured_qdot = [0.0, 0.0, 0.0]
        self.start_q = [0.0, 0.0, 0.0]
        self.target_q = [0.0, 0.0, 0.0]

        ##################### TRAJECTORY STATE #######################
        self.is_moving = False
        self.move_start_time = 0.0
        self._target_seq = 0          # increasing id for correlating log lines per target
        self._active_target = (0, (0.0, 0.0))  # (seq, xy) for the move currently in flight
        self._traj = {"kind": "position"}  # plan for the move currently in flight (see target_callback)

        ##################### ROS 2 INTERFACES #######################
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

        ##################### CONTROL LOOP #######################
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(f"[STARTUP] controller_node ready -- publish_rate={self.publish_rate}Hz, "
                               f"control_mode={self.control_mode}, trajectory_duration={self.duration}s, "
                               f"max_joint_velocity={self.max_joint_velocity}rad/s, "
                               f"max_joint_acceleration={self.max_joint_acceleration}rad/s^2, "
                               f"link_lengths={LINK_LENGTHS}")

    def target_callback(self, msg: Point):
        """Triggered when a new target (X, Y) is received."""
        target_xy = (msg.x, msg.y)
        self._target_seq += 1
        seq = self._target_seq
        self.get_logger().info(f"[RECEIVED] #{seq} target=({target_xy[0]:.3f}, {target_xy[1]:.3f})")

        # 0. Work out (ourselves, without modifying planar_arm.py) whether
        #    the requested target lies outside the workspace, so we can
        #    tell the GUI *why* the arm is heading somewhere unexpected.
        #    reachable_target() is part of the library's public API, so
        #    calling it here is fine -- we're just not editing the file.
        max_reach = sum(self.arm.link_lengths)
        requested = np.asarray(target_xy, dtype=float)
        clamped_target = self.arm.reachable_target(target_xy)
        was_clamped = np.linalg.norm(requested) > max_reach

        if was_clamped:
            self.get_logger().warn(
                f"[WARNING] #{seq} target outside workspace (reach={max_reach:.2f}) "
                f"-- clamping ({target_xy[0]:.2f}, {target_xy[1]:.2f}) -> "
                f"({clamped_target[0]:.2f}, {clamped_target[1]:.2f})"
            )

        # 1. Run IK on the incoming target
        try:
            solution = self.arm.inverse_kinematics(target_xy, initial_guess=self.current_q)
        except Exception as e:
            self.get_logger().error(f"[FAILED] #{seq} IK raised an exception: {e}")
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
                f"[FAILED] #{seq} IK solution for ({target_xy[0]:.2f}, {target_xy[1]:.2f}) "
                f"violates joint limits or ground constraint -- holding current position."
            )
            self.status_pub.publish(String(data="REJECTED:constraint_violation"))
            return

        # 3. Setup trajectory interpolation variables
        self.start_q = list(self.current_q)
        self.target_q = solution
        self.move_start_time = time.time()
        self.is_moving = True
        self._active_target = (seq, target_xy)
        self.goal_pub.publish(Bool(data=False))

        # 3b. Build the plan for this move under the active control mode.
        #     "position": original fixed-duration quintic blend.
        #     "velocity": trapezoidal, velocity/acceleration-limited profile
        #     -- duration falls out of the physics instead of being preset,
        #     which is the point of a velocity-controlled move.
        if self.control_mode == "velocity":
            deltas = [self.target_q[i] - self.start_q[i] for i in range(3)]
            distance = max(abs(d) for d in deltas)
            t_accel, t_cruise, t_total, v_peak = self._trapezoidal_timing(
                distance, self.max_joint_velocity, self.max_joint_acceleration
            )
            self._traj = {
                "kind": "velocity",
                "t_accel": t_accel, "t_cruise": t_cruise, "t_total": t_total,
                "v_peak": v_peak, "distance": distance,
            }
            duration_used = t_total
        else:
            self._traj = {"kind": "position"}
            duration_used = self.duration

        # 4. Tell the GUI whether the target was reachable as-is or had to
        #    be clamped to the workspace boundary -- this is what
        #    surfaces the (7.0, 3.0) edge case sensibly instead of just
        #    printing to the controller's own stdout.
        if was_clamped:
            status = (
                f"CLAMPED:requested=({target_xy[0]:.2f},{target_xy[1]:.2f}) "
                f"used=({clamped_target[0]:.2f},{clamped_target[1]:.2f}) mode={self.control_mode}"
            )
        else:
            status = f"OK:target=({target_xy[0]:.2f},{target_xy[1]:.2f}) mode={self.control_mode}"
        self.status_pub.publish(String(data=status))

        deg = [f"{np.degrees(a):.2f}" for a in solution]
        self.get_logger().info(
            f"[MOVING] #{seq} mode={self.control_mode} IK solved -- joints -> "
            f"[{deg[0]}, {deg[1]}, {deg[2]}] deg over {duration_used:.2f}s"
        )

    @staticmethod
    def _trapezoidal_timing(distance, v_max, a_max):
        """Timing for a trapezoidal (accel / cruise / decel) velocity profile
        covering `distance` at most `v_max`, ramping at most `a_max`.
        Falls back to a triangular profile (never reaching v_max) for short
        moves. Returns (t_accel, t_cruise, t_total, v_peak)."""
        if distance <= 1e-9:
            return 0.0, 0.0, 1e-6, 0.0
        t_accel = v_max / a_max
        accel_dist = 0.5 * a_max * t_accel ** 2
        if 2 * accel_dist >= distance:
            t_accel = math.sqrt(distance / a_max)
            v_peak = a_max * t_accel
            t_cruise = 0.0
        else:
            v_peak = v_max
            t_cruise = (distance - 2 * accel_dist) / v_max
        return t_accel, t_cruise, 2 * t_accel + t_cruise, v_peak

    @staticmethod
    def _trapezoidal_eval(t, t_accel, t_cruise, t_total, v_peak, a_max, distance):
        """Distance covered and velocity at time t along a trapezoidal profile
        built by _trapezoidal_timing (same args, plus t and distance)."""
        if t <= 0.0:
            return 0.0, 0.0
        if t >= t_total:
            return distance, 0.0
        if t <= t_accel:
            return 0.5 * a_max * t * t, a_max * t
        if t <= t_accel + t_cruise:
            accel_dist = 0.5 * a_max * t_accel * t_accel
            return accel_dist + v_peak * (t - t_accel), v_peak
        remaining = t_total - t
        return distance - 0.5 * a_max * remaining * remaining, a_max * remaining

    def control_loop(self):
        """Steps along the trajectory and publishes JointState[cite: 2]."""

        if self.is_moving:
            elapsed = time.time() - self.move_start_time
            kind = self._traj["kind"]

            if kind == "velocity":
                t_total = self._traj["t_total"]
                distance = self._traj["distance"]
                reached = elapsed >= t_total
                s_dist, v_dist = self._trapezoidal_eval(
                    min(elapsed, t_total), self._traj["t_accel"], self._traj["t_cruise"],
                    t_total, self._traj["v_peak"], self.max_joint_acceleration, distance
                )
                s = (s_dist / distance) if distance > 1e-9 else 1.0
                s_dot_norm = (v_dist / distance) if distance > 1e-9 else 0.0
            else:
                t = min(elapsed / self.duration, 1.0)
                reached = elapsed >= self.duration
                # Quintic blend: s(t) = 10t^3 - 15t^4 + 6t^5 gives zero
                # velocity/acceleration at both ends; s_dot is its time
                # derivative, scaled back into per-second units.
                s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)
                s_dot_norm = (30 * t ** 2 - 60 * t ** 3 + 30 * t ** 4) / self.duration

            for i in range(3):
                delta = self.target_q[i] - self.start_q[i]
                self.current_q[i] = self.start_q[i] + s * delta
                self.current_qdot[i] = s_dot_norm * delta

            if reached:
                self.is_moving = False
                self.current_qdot = [0.0, 0.0, 0.0]
                seq, target_xy = self._active_target
                actual = self.arm.end_effector(self.current_q)
                self.get_logger().info(
                    f"[REACHED] #{seq} mode={self.control_mode} target=({target_xy[0]:.2f}, {target_xy[1]:.2f}) "
                    f"actual_ee=({actual[0]:.3f}, {actual[1]:.3f}) after {elapsed:.2f}s"
                )
                self.goal_pub.publish(Bool(data=True))

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
        msg.velocity = self.current_qdot
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
        self.measured_qdot = list(self.current_qdot)

    def publish_state(self):
        """Constructs and publishes the (measured) JointState message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.measured_q
        msg.velocity = self.measured_qdot
        self.state_pub.publish(msg)


##################### FUNCTION DEFINITION #######################

def main(args=None):
    '''
    Description: Initializes ROS 2, creates the controller node, and spins
                 until shutdown.
    '''
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