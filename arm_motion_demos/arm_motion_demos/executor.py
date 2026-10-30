"""
Execute validated motions on the arm and gripper trajectory controllers.

Shared by motion_demo and the arm motion player. A RobotExecutor attaches
to any rclpy node: it follows /joint_states, publishes a Motion to the four
JointTrajectoryControllers (one trajectory each, started at the same stamp),
and compares /joint_states with the plan while it runs. If a joint leaves the
plan, gets close to a URDF limit, /joint_states stops, or the caller asks to
stop, every controller is brought to rest and Aborted is raised.

The controllers report success without checking their goal, so success here
always means: /joint_states reached the final pose.
"""
import time

from ament_index_python.packages import get_package_share_directory
from arm_motion_demos import motions as m
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import xacro
import yaml

CONTROLLERS = {
    'arm_a_controller': m.ARM_A,
    'arm_b_controller': m.ARM_B,
    'gripper_a_controller': [m.GRIPPERS[0]],
    'gripper_b_controller': [m.GRIPPERS[1]],
}
START_DELAY = 0.5          # s of controller time between publishing and starting
HOME_TOLERANCE = 0.005     # rad, "already at home" (a robot left on a limit gets homed)
GOAL_TOLERANCE = 0.02      # rad, final pose reached
LIMIT_GUARD = 0.003        # rad, stop if a joint gets this close to a URDF limit
MOVING_VELOCITY = 0.05     # rad/s, robot considered moving
COLLISION_STEP = 0.1       # s between collision-checked states


def load_robot():
    """(limits, home) from the installed URDF xacro, joint_limits.yaml and SRDF."""
    desc = get_package_share_directory('arm_body_description')
    moveit = get_package_share_directory('arm_body_moveit_config')
    urdf = xacro.process_file(f'{desc}/urdf/arm_body.urdf.xacro').toxml()
    with open(f'{moveit}/config/joint_limits.yaml') as f:
        limits = m.parse_limits(urdf, yaml.safe_load(f))
    with open(f'{moveit}/config/arm_body.srdf') as f:
        home = m.parse_home(f.read())
    if m.validate(m.Motion(np.zeros(1), home[None, :], []), limits):
        raise m.MotionError('SRDF home is outside the safe joint limits')
    return limits, home


class Aborted(Exception):
    """Execution stopped; the controllers were told to hold."""


class Stopped(Aborted):
    """Execution stopped on request (should_stop); the controllers hold."""


class RobotExecutor:
    """
    Joint state, collision check and monitored execution, on `node`.

    `log` (anything with info/warn methods) replaces the node's logger, e.g.
    for plain terminal output.
    """

    def __init__(self, node, limits, log=None):
        self.node, self.limits = node, limits
        self.log = log or node.get_logger()
        self.pubs = {c: node.create_publisher(JointTrajectory, f'/{c}/joint_trajectory', 10)
                     for c in CONTROLLERS}
        self.state = None  # (stamp, positions (14,), velocities (14,))
        self.state_wall = 0.0  # time.monotonic() when it arrived
        node.create_subscription(JointState, '/joint_states', self._on_joint_state, 50)
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')

    def _on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in m.JOINTS):
            return
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        self.state = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                      np.array([pos[j] for j in m.JOINTS]),
                      np.array([vel.get(j, 0.0) for j in m.JOINTS]))
        self.state_wall = time.monotonic()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # --- readiness ---------------------------------------------------------------
    def missing(self):
        """List what is not available yet: controllers without subscriber, /joint_states."""
        out = [c for c, p in self.pubs.items() if p.get_subscription_count() == 0]
        if self.state is None:
            out.append('/joint_states')
        return out

    def wait_for_robot(self, timeout=15.0):
        """[] when every controller listens and /joint_states arrives, else what is missing."""
        end = time.monotonic() + timeout
        while self.missing() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.missing()

    def is_moving(self):
        return self.state is not None and np.abs(self.state[2]).max() > MOVING_VELOCITY

    def check_collisions(self, name, motion):
        """First colliding state of `motion` per MoveIt, or None if all are free."""
        step = max(1, int(round(COLLISION_STEP / m.DT)))
        indices = sorted(set(range(0, len(motion.t), step)) | {len(motion.t) - 1})
        for i in indices:
            req = GetStateValidity.Request()
            req.group_name = 'both_arms'
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(m.JOINTS)
            req.robot_state.joint_state.position = [float(v) for v in motion.q[i]]
            future = self.validity.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            res = future.result()
            if res is None:
                return f'{name}: /check_state_validity did not answer'
            if not res.valid:
                pairs = sorted({f'{c.contact_body_1}-{c.contact_body_2}'
                                for c in res.contacts})
                return f'{name}: collision at t={motion.t[i]:.2f} s ({", ".join(pairs)})'
        return None

    # --- execution ---------------------------------------------------------------
    def hold(self, reason):
        """
        Bring every joint to rest near where it is, and hold there.

        The controllers reject an empty trajectory ("Empty trajectory
        received") and keep going, so this sends one point: the current
        position plus the stopping distance v*tau/2, reached at rest after
        tau = |v| / (0.8 * acceleration limit), clamped to the safe band.
        """
        if self.state is None:
            self.log.warn(f'Not moving, nothing to hold: {reason}')
            return
        _, q, v = self.state
        tau = max(0.2, float(np.max(np.abs(v) / (0.8 * self.limits.acceleration))))
        target = np.clip(q + 0.5 * v * tau, self.limits.safe_lower, self.limits.safe_upper)
        for c, joints in CONTROLLERS.items():
            idx = [m.INDEX[j] for j in joints]
            point = JointTrajectoryPoint(positions=[float(x) for x in target[idx]],
                                         velocities=[0.0] * len(idx),
                                         accelerations=[0.0] * len(idx))
            point.time_from_start.sec = int(tau)
            point.time_from_start.nanosec = int((tau - int(tau)) * 1e9)
            self.pubs[c].publish(JointTrajectory(joint_names=joints, points=[point]))
        self.log.warn(f'Holding position (stopping in {tau:.2f} s): {reason}')
        self.spin(0.3)  # let the messages go out before a possible shutdown

    def publish(self, motion, start_stamp):
        times, pos, vel, acc = motion.waypoints()
        sec = int(start_stamp)
        for c, joints in CONTROLLERS.items():
            idx = [m.INDEX[j] for j in joints]
            msg = JointTrajectory(joint_names=joints)
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int((start_stamp - sec) * 1e9)
            for k in range(1, len(times)):  # t=0 is the current pose
                p = JointTrajectoryPoint()
                p.positions = [float(v) for v in pos[k, idx]]
                p.velocities = [float(v) for v in vel[k, idx]]
                p.accelerations = [float(v) for v in acc[k, idx]]
                t = float(times[k])
                p.time_from_start.sec = int(t)
                p.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
                msg.points.append(p)
            self.pubs[c].publish(msg)

    def execute(self, name, motion, guard_limits=True, tolerance=0.2,
                should_stop=None, on_sample=None, goal_tolerance=GOAL_TOLERANCE,
                limit_guard=LIMIT_GUARD):
        """
        Run one motion and return the largest deviation from the plan (rad).

        should_stop() is polled while moving; a non-empty return value stops
        the motion (Stopped). on_sample(t, measured, planned) is called for
        every /joint_states message after the start. The motion is done when
        /joint_states is within goal_tolerance (rad) of its final pose; real
        servos stop a little short under load. With guard_limits, a joint
        within limit_guard (rad) of a URDF limit stops the motion. Raises
        Aborted (after holding) if anything goes wrong.
        """
        start = self.state[0] + START_DELAY
        self.publish(motion, start)
        self.log.info(f'[{name}] start ({motion.duration:.1f} s)')
        phases = list(motion.phases)
        lower = self.limits.lower + limit_guard
        upper = self.limits.upper - limit_guard
        last_stamp, last_wall, worst = None, time.monotonic(), 0.0
        settle_end = None
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            reason = should_stop() if should_stop else None
            if reason:
                self.hold(reason)
                raise Stopped(f'{name}: {reason}')
            stamp, q, _ = self.state
            if stamp == last_stamp:
                if time.monotonic() - last_wall > 2.0:
                    self.hold('/joint_states stopped')
                    raise Aborted(f'{name}: /joint_states stopped')
                continue
            last_stamp, last_wall = stamp, time.monotonic()
            t = stamp - start
            if t < 0:
                continue
            planned = motion.sample(t)
            if on_sample:
                on_sample(t, q, planned)
            while phases and phases[0][0] <= t:
                self.log.info(f'[{name}] {phases.pop(0)[1]}')
            err = np.abs(q - planned)
            worst = max(worst, float(err.max()))
            if err.max() > tolerance:
                j = m.JOINTS[int(err.argmax())]
                self.hold(f'{j} is {err.max():.3f} rad off the plan')
                raise Aborted(f'{name}: {j} deviated {err.max():.3f} rad from the plan '
                              f'at t={t:.2f} s (tolerance {tolerance})')
            if guard_limits and ((q < lower) | (q > upper)).any():
                j = m.JOINTS[int(np.argmax((lower - q).clip(0) + (q - upper).clip(0)))]
                self.hold(f'{j} at {q[m.INDEX[j]]:+.4f} is too close to its limit')
                raise Aborted(f'{name}: {j} came within {limit_guard} rad of its limit')
            if t >= motion.duration:
                final = np.abs(q - motion.q[-1]).max()
                if final < goal_tolerance:
                    break
                settle_end = settle_end or t + 3.0
                if t > settle_end:
                    self.hold('final pose not reached')
                    raise Aborted(f'{name}: final pose not reached ({final:.3f} rad off)')
        self.log.info(f'[{name}] done, max deviation from plan {worst:.3f} rad')
        return worst
