"""Bridge between MoveIt and the real Feetech arm.

Publishes /joint_states for the whole robot and serves the FollowJointTrajectory actions
MoveIt executes plans on. Servo joints are read from and commanded on the bus; joints without
a servo (config `simulated`) hold their value and follow trajectories instantly.

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

from arm_body_hardware import config
from arm_body_hardware.feetech import FeetechBus

MAX_READ_FAILURES = 10
LIMIT_SLACK = 0.02  # rad
VELOCITY_WINDOW = 0.25  # s


class Trajectory:
    """A trajectory being played on one controller's joints."""

    def __init__(self, joints, times, positions):
        self.joints = joints
        self.times = times          # (n,) seconds from start
        self.positions = positions  # (n, len(joints))
        self.start = time.monotonic()
        self.finished = False       # sampling reached the last point
        self.failed = None          # error string if the bridge stopped it

    def sample(self, now):
        t = now - self.start
        if t >= self.times[-1]:
            self.finished = True
            return self.positions[-1]
        return np.array([np.interp(t, self.times, self.positions[:, k])
                         for k in range(len(self.joints))])


class FeetechBridge(Node):

    def __init__(self):
        super().__init__('feetech_bridge')
        self.declare_parameter('config', '')
        self.declare_parameter('monitor', False)
        self.declare_parameter('torque_off_on_exit', False)
        self.monitor = self.get_parameter('monitor').value

        cfg = config.load_config(self.get_parameter('config').value or None)
        self.cfg = cfg
        self.servos = cfg['servos']
        self.zero_raw = config.load_calibration(cfg)
        self.limits = config.urdf_limits()
        self.ids = [s['id'] for s in self.servos.values()]

        self.lock = threading.Lock()
        self.pos = {j: float(v) for j, v in cfg['simulated'].items()}
        self.read_failures = {j: 0 for j in self.servos}
        self.active = {}  # controller name -> Trajectory
        self.history = collections.deque()  # (time, positions) for the velocity window

        self.bus = FeetechBus(cfg['port'], cfg['baud'])
        self._read_servos()
        unread = [j for j in self.servos if j not in self.pos]
        if unread:
            raise RuntimeError(f'No answer from {unread}. Is the arm powered?')

        if self.monitor:
            self.bus.set_torque(self.ids, False)
            self.get_logger().warn('MONITOR mode: torque off, trajectories are refused.')
        else:
            # Hold where the arm is now, so enabling torque does not make it jump
            self.bus.sync_write_positions(
                {s['id']: self._raw(j, self.pos[j]) for j, s in self.servos.items()})
            self.bus.set_torque(self.ids, True)
            self.get_logger().info('Torque on. Holding the current pose.')

        self.joint_names = list(self.servos) + list(cfg['simulated'])
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
        self.get_logger().info(f'Ready: {", ".join(cfg["controllers"])}')

    # --- bus ------------------------------------------------------------------------------

    def _raw(self, joint, rad):
        return config.to_raw(rad, self.zero_raw[joint], self.servos[joint]['sign'])

    def _read_servos(self):
        for joint, s in self.servos.items():
            raw = self.bus.read_position(s['id'])
            if raw is None:
                self.read_failures[joint] += 1
                continue
            self.read_failures[joint] = 0
            rad = config.to_rad(raw, self.zero_raw[joint], s['sign'])
            # A servo resting on a URDF limit (joint2 hanging at 0) reads a tick or two past it,
            # and MoveIt refuses to plan from out-of-bounds. Larger excursions are left visible.
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
            goals = {}
            now = time.monotonic()
            for traj in self.active.values():
                if lost:
                    traj.failed = f'lost contact with {lost}'
                if traj.failed:
                    continue
                for joint, value in zip(traj.joints, traj.sample(now)):
                    if joint in self.servos:
                        goals[self.servos[joint]['id']] = self._raw(joint, value)
                    else:
                        self.pos[joint] = float(value)
            if lost and self.active:
                self.get_logger().error(f'Lost contact with {lost}; stopping trajectories.')
            self.bus.sync_write_positions(goals)

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

    # --- actions --------------------------------------------------------------------------

    def _on_goal(self, _request):
        return GoalResponse.REJECT if self.monitor else GoalResponse.ACCEPT

    def _execute(self, goal_handle, name, ctrl):
        result = FollowJointTrajectory.Result()
        traj_msg = goal_handle.request.trajectory
        joints = list(traj_msg.joint_names)

        def abort(code, why):
            self.get_logger().error(f'{name}: {why}')
            result.error_code, result.error_string = code, why
            goal_handle.abort()
            return result

        unknown = set(joints) - set(ctrl['joints'])
        if unknown or not traj_msg.points:
            return abort(FollowJointTrajectory.Result.INVALID_JOINTS,
                         f'joints {sorted(unknown)} are not on this controller' if unknown
                         else 'empty trajectory')

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
            return abort(FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                         f'trajectory starts {start_err:.2f} rad from the real arm')
        if times[0] > 0.0:
            times = np.concatenate([[0.0], times])
            positions = np.vstack([current, positions])

        traj = Trajectory(joints, times, positions)
        with self.lock:
            old = self.active.get(name)
            if old is not None:
                old.failed = 'preempted by a new goal'
            self.active[name] = traj

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
            self.bus.sync_write_positions({self.servos[j]['id']: self._raw(j, self.pos[j])
                                           for j in joints if j in self.servos})

    def shutdown(self):
        if self.get_parameter('torque_off_on_exit').value:
            self.bus.set_torque(self.ids, False)
            self.get_logger().warn('Torque off.')
        self.bus.close()


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
