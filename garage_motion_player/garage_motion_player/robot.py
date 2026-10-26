"""
The player's single point of contact with the robot.

Every motion goes through Robot.play(), which runs one motion at a time and
only after: the limit validator passed, MoveIt's collision check passed, the
robot is not moving, and the plan starts where the robot really is. Execution
itself is the shared arm_motion_demos RobotExecutor (trajectory controllers,
monitored against /joint_states). Going home uses the direct joint path when
MoveIt finds it collision-free, otherwise MoveIt plans around the obstacle.
"""
import time
from typing import Callable, Dict, List, Optional, Sequence

from arm_motion_demos import motions as m
from arm_motion_demos.executor import CONTROLLERS, load_robot, RobotExecutor, Stopped
from garage_motion_player import dof_motions
from garage_motion_player.live_jog import LiveJogger, stream_points
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MoveItErrorCodes,
                             PlanningSceneComponents)
from moveit_msgs.srv import GetPlanningScene
import numpy as np
import rclpy
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

AT_HOME = 0.01      # rad: every joint this close to home counts as "at home"
START_MATCH = 0.03  # rad: a plan must start this close to the measured pose
SETTLE_SLACK = 0.003  # rad: Gazebo settles up to ~1 mrad off a command (e.g. 0.0097 for 0.01)


class PlainLog:
    """Executor messages as plain indented lines (no ROS log prefixes)."""

    def __init__(self, out: Callable[[str], None]):
        self.out = out

    def info(self, msg):
        self.out(f'    {msg}')

    def warn(self, msg):
        self.out(f'  ! {msg}')

    error = warn


class Robot:
    def __init__(self, node, out: Callable[[str], None], tracking_tolerance: float = 0.2):
        self.node, self.out = node, out
        self.limits, self.home = load_robot()
        self.ex = RobotExecutor(node, self.limits, log=PlainLog(out))
        self.move_group = ActionClient(node, MoveGroup, '/move_action')
        self.scene = node.create_client(GetPlanningScene, '/get_planning_scene')
        self.tolerance = tracking_tolerance
        self.busy = False
        self._floor = False

    # --- state -------------------------------------------------------------------
    def missing(self) -> List[str]:
        """Parts not available yet (controllers, /joint_states, MoveIt)."""
        out = self.ex.missing()
        if not self.ex.validity.service_is_ready():
            out.append('MoveIt (/check_state_validity)')
        elif not self._floor_in_scene():
            out.append("Gazebo's floor in MoveIt's planning scene")
        return out

    def _floor_in_scene(self) -> bool:
        """Is ground_plane (add_ground_plane.py) in the planning scene? Cached once true."""
        if not self._floor and self.scene.service_is_ready():
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
            future = self.scene.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            res = future.result()
            self._floor = res is not None and any(
                o.id == 'ground_plane' for o in res.scene.world.collision_objects)
        return self._floor

    def positions(self) -> np.ndarray:
        """Return the latest measured joint angles, once pending messages are read."""
        rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.ex.state is None:
            raise m.MotionError('no /joint_states received')
        return self.ex.state[1].copy()

    def planning_start(self) -> np.ndarray:
        """
        Measured pose to plan from, nudged into the safe band if it is just outside.

        Gazebo holds a joint commanded to 0.01 rad (home, 0.01 inside a limit at 0)
        at about 0.0097. Planning from that exact value would put the plan's first
        point outside the safe band, so joints within SETTLE_SLACK of the band are
        planned from the band edge. Further outside is a real problem: left alone,
        so validation refuses it (go home first).
        """
        q = self.positions()
        lo, hi = self.limits.safe_lower, self.limits.safe_upper
        near = ((q < lo) & (q >= lo - SETTLE_SLACK)) | ((q > hi) & (q <= hi + SETTLE_SLACK))
        return np.where(near, np.clip(q, lo, hi), q)

    def wait_until_still(self, timeout: float = 5.0) -> bool:
        """Wait (e.g. after a stop, which decelerates) until no joint moves; False on timeout."""
        end = time.monotonic() + timeout
        while self.ex.is_moving() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return not self.ex.is_moving()

    def near(self, target, tolerance: float = 0.02) -> bool:
        return bool(np.abs(self.positions() - target).max() < tolerance)

    def at_home(self) -> bool:
        return bool(np.abs(self.positions() - self.home).max() < AT_HOME)

    # --- motions -----------------------------------------------------------------
    def play(self, name: str, motion: m.Motion, allow_start: bool = False,
             collision_check: bool = True, guard_limits: bool = True,
             should_stop=None, on_sample=None) -> float:
        """Check and run one motion; returns the largest deviation from the plan."""
        if self.busy:
            raise m.MotionError('another motion is still running')
        problems = m.validate(motion, self.limits, allow_start=allow_start)
        if problems:
            raise m.MotionError(f'{name} breaks the joint limits: ' + '; '.join(problems))
        if collision_check:
            problem = self.ex.check_collisions(name, motion)
            if problem:
                raise m.MotionError(problem)
        if not self.wait_until_still():
            raise m.MotionError('the robot is still moving (another program commanding it?)')
        off = np.abs(self.positions() - motion.q[0])
        if off.max() > START_MATCH:
            raise m.MotionError(f'{name} was planned from another pose '
                                f'({m.JOINTS[int(off.argmax())]} {off.max():.3f} rad off)')
        self.busy = True
        try:
            return self.ex.execute(name, motion, guard_limits=guard_limits,
                                   tolerance=self.tolerance, should_stop=should_stop,
                                   on_sample=on_sample)
        finally:
            self.busy = False

    def dof(self, joints: Sequence[str], amplitude: float, speed: float,
            should_stop=None, on_sample=None) -> List[str]:
        """
        Move only `joints` (one joint, or the same joint on both arms) and back.

        Each side of the swing is collision-checked on its own; unsafe sides
        are skipped. Returns notes about skipped sides.
        """
        start = self.planning_start()
        notes, safe = [], []
        for side in dof_motions.plan_sides(start, joints, amplitude, self.limits):
            motion = dof_motions.side_motion(start, side, self.limits, speed)
            problem = self.ex.check_collisions(side.label, motion)
            if problem:
                notes.append(f'skipped {problem}')
            else:
                safe.append(side)
        if not safe:
            raise m.MotionError('every direction would collide: ' + '; '.join(notes))
        name = '+'.join(joints)
        motion = dof_motions.combine(start, safe, self.limits, speed, name)
        self.play(name, motion, collision_check=False,  # each side was checked above
                  should_stop=should_stop, on_sample=on_sample)
        return notes

    def jog(self, joints: Sequence[str], speed: float, delta=None, goal=None,
            should_stop=None, on_sample=None) -> List[str]:
        """
        Move only `joints` by `delta` rad or to `goal` rad (the user's jog step).

        Same checks as every motion (limits, collision, still, start match);
        returns notes about joints stopped at their safe limit.
        """
        start = self.planning_start()
        target, notes = dof_motions.jog_target(start, joints, self.limits, delta, goal)
        motion = dof_motions.jog_motion(start, target, self.limits, speed)
        self.play('+'.join(joints), motion, should_stop=should_stop, on_sample=on_sample)
        return notes

    def goto(self, name: str, target, speed: float, should_stop=None, on_sample=None) -> None:
        """Move every joint to `target` (validated and collision-checked first)."""
        motion = m.MotionBuilder(self.planning_start(), self.limits, speed).move_to(
            target, label=name).build(name)
        self.play(name, motion, should_stop=should_stop, on_sample=on_sample)

    def go_home(self, speed: float, should_stop=None, on_sample=None) -> str:
        """Bring the robot home and confirm it from /joint_states."""
        self.wait_until_still()
        start = self.positions()
        if np.abs(start - self.home).max() < AT_HOME:
            return 'already at home'
        homing = m.home_motion(start, self.home, self.limits, speed)
        problem = self.ex.check_collisions('home', homing)
        if problem is None:
            self.play('home', homing, allow_start=True, collision_check=False,
                      guard_limits=False, should_stop=should_stop, on_sample=on_sample)
            how = 'direct path'
        else:
            self.out(f'  ! direct path blocked ({problem}); planning with MoveIt')
            self._moveit_home(speed, should_stop)
            how = 'MoveIt plan'
        self.ex.spin(0.5)
        err = np.abs(self.positions() - self.home)
        if err.max() >= AT_HOME:
            raise m.MotionError(f'home not reached ({m.JOINTS[int(err.argmax())]} '
                                f'{err.max():.3f} rad off)')
        return f'at home ({how})'

    def _moveit_home(self, speed: float, should_stop) -> None:
        """Arms home with a collision-aware MoveIt plan, then the grippers directly."""
        if not self.move_group.wait_for_server(timeout_sec=5.0):
            raise m.MotionError('MoveIt /move_action not available')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'both_arms'
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.5 * speed
        arm_joints = [j for j in m.JOINTS if j not in m.GRIPPERS]
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(self.home[m.INDEX[j]]),
                            tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
            for j in arm_joints])]
        goal.planning_options.plan_only = False
        self.busy = True
        try:
            send = self.move_group.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
            handle = send.result()
            if handle is None or not handle.accepted:
                raise m.MotionError('MoveIt did not accept the home goal')
            result = handle.get_result_async()
            end = time.monotonic() + 180.0
            while not result.done():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                reason = should_stop() if should_stop else None
                if reason or time.monotonic() > end:
                    handle.cancel_goal_async()
                    self.ex.hold(reason or 'MoveIt home timed out')
                    raise Stopped(f'home: {reason or "timed out"}')
            code = result.result().result.error_code.val
            if code != MoveItErrorCodes.SUCCESS:
                raise m.MotionError(f'MoveIt could not bring the arms home (error {code})')
        finally:
            self.busy = False
        start = self.positions()  # now the grippers
        target = start.copy()
        for g in m.GRIPPERS:
            target[m.INDEX[g]] = self.home[m.INDEX[g]]
        if np.abs(target - start).max() > 1e-3:
            grip = m.MotionBuilder(start, self.limits, speed).move_to(target).build('grippers')
            self.play('grippers home', grip, allow_start=True, guard_limits=False,
                      should_stop=should_stop)

    def stop(self, reason: str) -> None:
        self.ex.hold(reason)

    # --- live control ------------------------------------------------------------
    def state(self):
        """Return (sim stamp, measured positions) of the latest /joint_states, or None."""
        return None if self.ex.state is None else (self.ex.state[0], self.ex.state[1])

    def state_age(self) -> float:
        return time.monotonic() - self.ex.state_wall

    def collision(self, q) -> Optional[str]:
        """Check one pose with MoveIt: None if it is free, else what collides."""
        return self.ex.check_collisions('ahead', m.Motion(np.zeros(1), np.asarray(q)[None, :],
                                                          []))

    def stream(self, reference, velocity, horizon: float, joints: Sequence[str],
               stamp: float) -> None:
        """
        Send the controllers of `joints` the reference and where it goes next.

        Stamped at the reference's own sim time `stamp`, three points: the
        reference now (moving at `velocity`), where it will be in `horizon` s,
        then at rest where the joint would brake to from there. Stamping it
        lets the controller continue at the reference's speed; an unstamped
        command restarts from the measured state and lags badly (measured:
        0.06-0.13 rad/s instead of 0.2, up to 0.23 rad behind).
        The controller rejects a trajectory that ends moving ("Velocity of last
        trajectory point ... is not zero"), and ending at rest means that if
        commands stop coming, the joint brakes by itself, onto the braking pose
        that was collision-checked (live_jog.stream_points). Each new command
        replaces the previous one (the controller blends from where it is), so
        streaming it every tick gives a smooth jog.
        """
        ahead, vel, rest, t_rest = stream_points(reference, velocity, horizon, self.limits)
        sec = int(stamp)
        for c, names in CONTROLLERS.items():
            if not set(names) & set(joints):
                continue
            idx = [m.INDEX[j] for j in names]
            now = JointTrajectoryPoint(positions=[float(x) for x in reference[idx]],
                                       velocities=[float(x) for x in vel[idx]])
            moving = JointTrajectoryPoint(positions=[float(x) for x in ahead[idx]],
                                          velocities=[float(x) for x in vel[idx]])
            moving.time_from_start.nanosec = int(horizon * 1e9)
            stop = JointTrajectoryPoint(positions=[float(x) for x in rest[idx]],
                                        velocities=[0.0] * len(idx))
            stop.time_from_start.sec = int(t_rest)
            stop.time_from_start.nanosec = int((t_rest - int(t_rest)) * 1e9)
            msg = JointTrajectory(joint_names=names, points=[now, moving, stop])
            msg.header.stamp.sec, msg.header.stamp.nanosec = sec, int((stamp - sec) * 1e9)
            self.ex.pubs[c].publish(msg)

    def _spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while True:
            rclpy.spin_once(self.node, timeout_sec=max(0.0, end - time.monotonic()))
            if time.monotonic() >= end:
                return

    def live(self, joint_names: Dict[str, List[str]], keys, show, first_gap: float,
             repeat_gap: float) -> str:
        """Run a live-control session (see live_jog); returns a summary."""
        if self.busy:
            raise m.MotionError('another motion is still running')
        if not self.wait_until_still():
            raise m.MotionError('the robot is still moving (another program commanding it?)')
        jogger = LiveJogger(self, joint_names, keys, show, tolerance=self.tolerance,
                            first_gap=first_gap, repeat_gap=repeat_gap, sleep=self._spin_for)
        self.busy = True
        try:
            worst = jogger.run()
        finally:
            self.busy = False
        return f'left live control (robot within {worst:.3f} rad of the command)'
