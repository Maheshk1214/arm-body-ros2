"""Bridge between MoveIt and the real Feetech arms.

Publishes /joint_states for the whole robot and serves the FollowJointTrajectory actions
MoveIt executes plans on. Like a ros2_control JointTrajectoryController, each controller also
takes trajectories on <controller>/joint_trajectory (motion_demo, arm_motion_player), started
at their header stamp, or at once if it is zero. Servo joints are read from and commanded on
their arm's bus; joints without a servo, and all joints of arms left out of `arms`, hold their
config `home` value and follow trajectories instantly.

monitor:=true keeps torque off and refuses trajectories: move the arm by hand and watch RViz
to check the calibration and each servo's sign before letting MoveIt drive it.
"""
import collections
import threading
import time

import numpy as np
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from arm_body_hardware import config
from arm_body_hardware.feetech import FeetechBus

MAX_READ_FAILURES = 10
LIMIT_SLACK = 0.02  # rad
VELOCITY_WINDOW = 0.25  # s


class Trajectory:
    """A trajectory being played on one controller's joints."""

    def __init__(self, joints, times, positions, start=None):
        self.joints = joints
        self.times = times          # (n,) seconds from start
        self.positions = positions  # (n, len(joints))
        self.start = time.monotonic() if start is None else start
        self.finished = False       # the trajectory's time has run out
        self.failed = None          # error string if the bridge stopped it

    def sample(self, now):
        t = now - self.start
        if t >= self.times[-1]:
            return self.positions[-1]
        return np.array([np.interp(t, self.times, self.positions[:, k])
                         for k in range(len(self.joints))])


class FeetechBridge(Node):

    def __init__(self):
        super().__init__('feetech_bridge')
        self.declare_parameter('config', '')
        self.declare_parameter('monitor', False)
        self.declare_parameter('torque_off_on_exit', False)
        self.declare_parameter('arms', '')  # comma-separated; empty: every arm in the config
        self.monitor = self.get_parameter('monitor').value

        cfg = config.load_config(self.get_parameter('config').value or None)
        self.cfg = cfg
        arms = [a.strip() for a in self.get_parameter('arms').value.split(',') if a.strip()]
        arms = arms or list(cfg['arms'])
        unknown = set(arms) - set(cfg['arms'])
        if unknown:
            raise RuntimeError(f'Unknown arms {sorted(unknown)}; '
                               f'the config has {list(cfg["arms"])}')

        self.servos = {}       # joint -> {id, sign, arm}
        self.arm_joints = {}   # arm -> its servo joints
        self.zero_raw = {}
        for arm in arms:
            self.arm_joints[arm] = list(cfg['arms'][arm]['servos'])
            for joint, s in cfg['arms'][arm]['servos'].items():
                self.servos[joint] = {**s, 'arm': arm}
            self.zero_raw.update(config.load_calibration(cfg['arms'][arm]))
        uncalibrated = [j for j in self.servos if j not in self.zero_raw]
        if uncalibrated:
            arm = self.servos[uncalibrated[0]]['arm']
            raise RuntimeError(f'{uncalibrated} not calibrated. Run '
                               f'`ros2 run arm_body_hardware calibrate --arm {arm}`, or leave '
                               f'the arm out with arms:=...')
        self.leads = {j: float(s.get('lead', cfg.get('command_lead', 0.0)))
                      for j, s in self.servos.items()}
        # A full-turn joint (wrist roll) is kept to the part of its range the servo reaches
        self.limits = config.urdf_limits()
        for joint, s in self.servos.items():
            self.limits[joint] = config.servo_limits(*self.limits[joint],
                                                     self.zero_raw[joint], s['sign'])

        self.lock = threading.Lock()
        self.pos = {j: float(v) for j, v in cfg['home'].items() if j not in self.servos}
        self.read_failures = {j: 0 for j in self.servos}
        self.active = {}  # controller name -> Trajectory
        self.history = collections.deque()  # (time, positions) for the velocity window

        self.buses = {arm: FeetechBus(cfg['arms'][arm]['port'], cfg['baud']) for arm in arms}
        self._read_servos()
        unread = [j for j in self.servos if j not in self.pos]
        if unread:
            raise RuntimeError(f'No answer from {unread}. Is the arm powered?')

        if self.monitor:
            self._set_torque(False)
            self.get_logger().warn('MONITOR mode: torque off, trajectories are refused.')
        else:
            # Hold where the arms are now, so enabling torque does not make them jump
            self._write({j: self.pos[j] for j in self.servos})
            self._set_torque(True)
            self.get_logger().info('Torque on. Holding the current pose.')

        self.joint_names = list(self.servos) + [j for j in cfg['home'] if j not in self.servos]
        self.state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / cfg['rate'], self._loop,
                          callback_group=MutuallyExclusiveCallbackGroup())

        actions = ReentrantCallbackGroup()
        for name, ctrl in cfg['controllers'].items():
            ActionServer(
                self, FollowJointTrajectory, f'{name}/follow_joint_trajectory',
                execute_callback=lambda gh, n=name, c=ctrl: self._execute(gh, n, c),
                goal_callback=self._on_goal,
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=actions)
            self.create_subscription(
                JointTrajectory, f'{name}/joint_trajectory',
                lambda msg, n=name, c=ctrl: self._on_trajectory(msg, n, c), 10,
                callback_group=actions)
        self.get_logger().info(f'Ready: {", ".join(cfg["controllers"])}. '
                               f'Real arms: {", ".join(arms)}')

    # --- bus ------------------------------------------------------------------------------

    def _raw(self, joint, rad):
        return config.to_raw(rad, self.zero_raw[joint], self.servos[joint]['sign'])

    def _write(self, targets):
        """targets: {servo joint: rad}, one sync write per arm's bus."""
        goals = {arm: {} for arm in self.buses}
        for joint, rad in targets.items():
            s = self.servos[joint]
            goals[s['arm']][s['id']] = self._raw(joint, rad)
        for arm, bus in self.buses.items():
            bus.sync_write_positions(goals[arm])

    def _set_torque(self, enable):
        for arm, bus in self.buses.items():
            bus.set_torque([self.servos[j]['id'] for j in self.arm_joints[arm]], enable)

    def _read_servos(self):
        for arm, bus in self.buses.items():
            raws = bus.sync_read_positions([self.servos[j]['id'] for j in self.arm_joints[arm]])
            for joint in self.arm_joints[arm]:
                s = self.servos[joint]
                raw = raws.get(s['id'])
                if raw is None:
                    self.read_failures[joint] += 1
                    continue
                self.read_failures[joint] = 0
                rad = config.to_rad(raw, self.zero_raw[joint], s['sign'])
                # A servo resting on a URDF limit (joint2 hanging at 0) reads a tick or two past
                # it, and MoveIt refuses to plan from out-of-bounds. Larger excursions are left
                # visible.
                lo, hi = self.limits[joint]
                if lo - LIMIT_SLACK <= rad < lo:
                    rad = lo
                elif hi < rad <= hi + LIMIT_SLACK:
                    rad = hi
                self.pos[joint] = rad

    def _loop(self):
        with self.lock:
            self._read_servos()
            lost = [j for j, n in self.read_failures.items() if n >= MAX_READ_FAILURES]
            targets = {}
            now = time.monotonic()
            for traj in self.active.values():
                if lost:
                    traj.failed = f'lost contact with {lost}'
                if traj.failed:
                    continue
                samples = {}  # lead -> positions of every joint of traj
                for k, joint in enumerate(traj.joints):
                    lead = self.leads.get(joint, 0.0)
                    if lead not in samples:
                        samples[lead] = traj.sample(now + lead)
                    if joint in self.servos:
                        targets[joint] = samples[lead][k]
                    else:
                        self.pos[joint] = float(samples[lead][k])
                traj.finished = now - traj.start >= traj.times[-1]
            if lost and self.active:
                self.get_logger().error(f'Lost contact with {lost}; stopping trajectories.')
            self._write(targets)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [self.pos[j] for j in self.joint_names]
        # Velocity over a window, not between two reads: one tick of jitter in 1/rate s
        # would look like motion to nodes that wait for the arm to be still
        self.history.append((now, msg.position))
        while len(self.history) > 2 and now - self.history[1][0] >= VELOCITY_WINDOW:
            self.history.popleft()
        t0, p0 = self.history[0]
        dt = now - t0
        msg.velocity = [(p - q) / dt if dt > 0 else 0.0 for p, q in zip(msg.position, p0)]
        self.state_pub.publish(msg)

    # --- trajectories -----------------------------------------------------------------------

    def _prepare(self, name, traj_msg, ctrl):
        """(joints, times, positions, current) checked and clipped, or an error string."""
        joints = list(traj_msg.joint_names)
        unknown = set(joints) - set(ctrl['joints'])
        if unknown:
            return f'joints {sorted(unknown)} are not on this controller'
        if not traj_msg.points:
            return 'empty trajectory'
        times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                          for p in traj_msg.points])
        positions = np.array([list(p.positions) for p in traj_msg.points], dtype=float)
        for k, joint in enumerate(joints):
            positions[:, k] = np.clip(positions[:, k], *self.limits[joint])
        with self.lock:
            current = np.array([self.pos[j] for j in joints])
        servo_cols = [k for k, j in enumerate(joints) if j in self.servos]
        start_err = np.abs(positions[0, servo_cols] - current[servo_cols]).max(initial=0.0)
        if start_err > self.cfg['max_start_error']:
            return f'trajectory starts {start_err:.2f} rad from the real arm'
        if times[0] > 0.0:
            times = np.concatenate([[0.0], times])
            positions = np.vstack([current, positions])
        return joints, times, positions, current

    def _start(self, name, traj):
        with self.lock:
            old = self.active.get(name)
            if old is not None:
                old.failed = 'preempted by a new goal'
            self.active[name] = traj

    def _on_trajectory(self, msg, name, ctrl):
        if self.monitor:
            self.get_logger().warn(f'{name}: MONITOR mode, trajectory ignored')
            return
        prepared = self._prepare(name, msg, ctrl)
        if isinstance(prepared, str):
            self.get_logger().error(f'{name}: {prepared}')
            return
        joints, times, positions, _ = prepared
        # The header stamp is on this node's clock (the one /joint_states is stamped with)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delay = 0.0
        if stamp > 0.0:
            delay = max(0.0, stamp - self.get_clock().now().nanoseconds * 1e-9)
        self._start(name, Trajectory(joints, times, positions, time.monotonic() + delay))

    # --- actions --------------------------------------------------------------------------

    def _on_goal(self, _request):
        return GoalResponse.REJECT if self.monitor else GoalResponse.ACCEPT

    def _execute(self, goal_handle, name, ctrl):
        result = FollowJointTrajectory.Result()

        def abort(code, why):
            self.get_logger().error(f'{name}: {why}')
            result.error_code, result.error_string = code, why
            goal_handle.abort()
            return result

        prepared = self._prepare(name, goal_handle.request.trajectory, ctrl)
        if isinstance(prepared, str):
            code = (FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED if 'starts' in prepared
                    else FollowJointTrajectory.Result.INVALID_JOINTS)
            return abort(code, prepared)
        joints, times, positions, _ = prepared

        traj = Trajectory(joints, times, positions)
        self._start(name, traj)

        try:
            while not traj.finished and traj.failed is None:
                if goal_handle.is_cancel_requested:
                    self._hold(joints)
                    goal_handle.canceled()
                    result.error_string = 'canceled'
                    return result
                time.sleep(0.01)
            if traj.failed:
                return abort(FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED, traj.failed)

            # Wait for the real joints to settle at the goal
            goal = dict(zip(joints, positions[-1]))
            tol = float(ctrl['goal_tolerance'])
            deadline = time.monotonic() + self.cfg['settle_time']
            while True:
                with self.lock:
                    err = max((abs(self.pos[j] - goal[j]) for j in joints if j in self.servos),
                              default=0.0)
                if err <= tol or time.monotonic() > deadline:
                    break
                time.sleep(0.02)
            if err > tol:
                return abort(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                             f'ended {err:.3f} rad from the goal (tolerance {tol})')
            goal_handle.succeed()
            return result
        finally:
            with self.lock:
                if self.active.get(name) is traj:
                    del self.active[name]

    def _hold(self, joints):
        """Stop where the arm is now."""
        with self.lock:
            for traj in self.active.values():
                if set(traj.joints) & set(joints):
                    traj.failed = 'canceled'
            self._write({j: self.pos[j] for j in joints if j in self.servos})

    def shutdown(self):
        if self.get_parameter('torque_off_on_exit').value:
            self._set_torque(False)
            self.get_logger().warn('Torque off.')
        for bus in self.buses.values():
            bus.close()


def main():
    rclpy.init()
    node = FeetechBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Bus first: a second Ctrl-C during ROS teardown must not skip torque-off / port close
        node.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
