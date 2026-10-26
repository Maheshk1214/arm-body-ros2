r"""
Run a sequence of gestures (wave, dual_arm, dance67) on the arm controllers.

    ros2 run arm_motion_demos motion_demo --ros-args \\
        -p sequence:="[wave, dual_arm, dance67]" -p speed:=1.0

Before anything moves, every gesture is built from the SRDF home and checked
against the URDF position/velocity limits and MoveIt's acceleration limits
(motions.validate), then for collisions with MoveIt's /check_state_validity
(the planning scene includes Gazebo's floor). While a gesture runs,
/joint_states is compared with the plan; on a large deviation, a joint near a
limit, lost /joint_states or Ctrl-C, all controllers are told to hold position.

Exit codes: 0 done, 1 failed while moving, 2 refused before moving,
130 interrupted.
"""
import sys
import time

from ament_index_python.packages import get_package_share_directory
from arm_motion_demos import motions as m
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
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


class MotionDemo(Node):
    def __init__(self):
        super().__init__('motion_demo')
        self.declare_parameter('sequence', ['wave', 'dual_arm', 'dance67'])
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('repetitions', 0)        # 0 = each gesture's default
        self.declare_parameter('wave_arm', 'arm_a')
        self.declare_parameter('dance_tempo', 54.0)     # beats/min, sim time
        self.declare_parameter('dance_amplitude', 0.25)  # rad of forearm tilt
        self.declare_parameter('collision_check', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('tracking_tolerance', 0.2)  # rad, plan vs /joint_states

        self.pubs = {c: self.create_publisher(JointTrajectory, f'/{c}/joint_trajectory', 10)
                     for c in CONTROLLERS}
        self.state = None  # (stamp, positions (14,), velocities (14,))
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.limits = None  # set by run(); hold() needs them

    def param(self, name):
        return self.get_parameter(name).value

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in m.JOINTS):
            return
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        self.state = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                      np.array([pos[j] for j in m.JOINTS]),
                      np.array([vel.get(j, 0.0) for j in m.JOINTS]))

    # --- setup -------------------------------------------------------------------
    def wait_for_robot(self, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.state is not None and all(
                    p.get_subscription_count() > 0 for p in self.pubs.values()):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = [c for c, p in self.pubs.items() if p.get_subscription_count() == 0]
        self.get_logger().error(
            f'Robot not available: controllers without subscriber {missing}, '
            f'/joint_states {"ok" if self.state else "missing"}')
        return False

    def check_collisions(self, name, motion):
        """First colliding state of `motion` per MoveIt, or None if all are free."""
        step = max(1, int(round(COLLISION_STEP / m.DT)))
        indices = list(range(0, len(motion.t), step)) + [len(motion.t) - 1]
        for i in indices:
            req = GetStateValidity.Request()
            req.group_name = 'both_arms'
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = list(m.JOINTS)
            req.robot_state.joint_state.position = [float(v) for v in motion.q[i]]
            future = self.validity.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
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
        if self.state is None or self.limits is None:
            self.get_logger().warn(f'Not moving, nothing to hold: {reason}')
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
        self.get_logger().warn(f'Holding position (stopping in {tau:.2f} s): {reason}')
        end = time.monotonic() + 0.3  # let the messages go out before shutting down
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

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

    def execute(self, name, motion, limits, guard_limits=True):
        """Run one motion; raises Aborted (after holding) if it goes wrong."""
        start = self.state[0] + START_DELAY
        self.publish(motion, start)
        self.get_logger().info(f'[{name}] start ({motion.duration:.1f} s)')
        phases = list(motion.phases)
        tol = self.param('tracking_tolerance')
        lower, upper = limits.lower + LIMIT_GUARD, limits.upper - LIMIT_GUARD
        last_stamp, last_wall, worst = None, time.monotonic(), 0.0
        settle_end = None
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
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
            while phases and phases[0][0] <= t:
                self.get_logger().info(f'[{name}] {phases.pop(0)[1]}')
            err = np.abs(q - motion.sample(t))
            worst = max(worst, float(err.max()))
            if err.max() > tol:
                j = m.JOINTS[int(err.argmax())]
                self.hold(f'{j} is {err.max():.3f} rad off the plan')
                raise Aborted(f'{name}: {j} deviated {err.max():.3f} rad from the plan '
                              f'at t={t:.2f} s (tolerance {tol})')
            if guard_limits and ((q < lower) | (q > upper)).any():
                j = m.JOINTS[int(np.argmax((lower - q).clip(0) + (q - upper).clip(0)))]
                self.hold(f'{j} at {q[m.INDEX[j]]:+.4f} is too close to its limit')
                raise Aborted(f'{name}: {j} came within {LIMIT_GUARD} rad of its limit')
            if t >= motion.duration:
                final = np.abs(q - motion.q[-1]).max()
                if final < GOAL_TOLERANCE:
                    break
                settle_end = settle_end or t + 3.0
                if t > settle_end:
                    self.hold('final pose not reached')
                    raise Aborted(f'{name}: final pose not reached ({final:.3f} rad off)')
        self.get_logger().info(f'[{name}] done, max deviation from plan {worst:.3f} rad')
        return worst

    def run(self):
        params = {k: self.param(k) for k in ('speed', 'wave_arm', 'dance_tempo',
                                             'dance_amplitude')}
        if self.param('repetitions'):
            params['repetitions'] = self.param('repetitions')
        names = list(self.param('sequence'))
        try:
            limits, home = load_robot()
            self.limits = limits
            sequence = m.build_sequence(names, home, limits, params)
        except m.MotionError as e:
            self.get_logger().error(f'Refusing to move: {e}')
            return 2
        total = sum(mo.duration for _, mo in sequence)
        self.get_logger().info(
            f'Sequence: home -> {" -> ".join(n for n, _ in sequence)} -> home '
            f'({total:.0f} s of controller time); all limit checks passed')
        if not self.wait_for_robot():
            return 2
        if self.param('collision_check'):
            if not self.validity.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(
                    'MoveIt /check_state_validity not available: start move_group '
                    '(demo.launch.py or moveit_gazebo.launch.py) or set collision_check:=false')
                return 2
            for name, motion in sequence:
                problem = self.check_collisions(name, motion)
                if problem:
                    self.get_logger().error(f'Refusing to move: {problem}')
                    return 2
            self.get_logger().info('Collision check passed (MoveIt planning scene)')
        else:
            self.get_logger().warn('Collision check disabled')
        if self.param('dry_run'):
            self.get_logger().info('Dry run: not moving')
            return 0
        _, q, v = self.state
        if np.abs(v).max() > MOVING_VELOCITY:
            self.get_logger().error('Robot is moving (is MoveIt executing?); not starting')
            return 2
        if np.abs(q - home).max() > HOME_TOLERANCE:
            homing = m.home_motion(q, home, limits, self.param('speed'))
            problems = m.validate(homing, limits, allow_start=True)
            if problems:
                self.get_logger().error('Cannot go home safely: ' + '; '.join(problems))
                return 2
            if self.param('collision_check'):
                problem = self.check_collisions('home', homing)
                if problem:
                    self.get_logger().error(f'Refusing to move: {problem}')
                    return 2
            sequence.insert(0, ('home', homing))
        for name, motion in sequence:
            self.execute(name, motion, limits, guard_limits=name != 'home')
        self.get_logger().info('Sequence finished at home')
        return 0


def main():
    # Python's own SIGINT handling, so the hold command can still be published
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MotionDemo()
    code = 1
    try:
        code = node.run()
    except Aborted as e:
        node.get_logger().error(f'Stopped: {e}')
        code = 1
    except KeyboardInterrupt:
        node.hold('interrupted (Ctrl-C)')
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
