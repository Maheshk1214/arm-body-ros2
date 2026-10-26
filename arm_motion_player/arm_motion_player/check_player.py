"""
Live check of the motion player: drives ./run.sh through its menu.

    ros2 run arm_motion_player check_player [--run-sh PATH] [--only menu|live]

Starts run.sh --headless, types menu choices when the prompts appear, and
records /joint_states with its own node, so the checks rest on what the
robot did, not on what the menu printed. Live control runs on a pseudo-
terminal with keys held like a keyboard (repeat after 0.5 s, 33 per second).
Uses the domain in player.yaml and needs nothing else running on it. The simulated robot moves.
"""
import argparse
import os
from pathlib import Path
import pty
import re
import signal
import subprocess
import sys
import time

from arm_motion_demos import motions as m
from arm_motion_demos.executor import load_robot
from arm_motion_player import simulation_manager as sm
from arm_motion_player.config import load_config
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

STARTED = []  # every Player, so main() can make sure none is left running
PROMPT = re.compile(r'(Choose: |Joint: |\[Y/n\] |Speed .*: |Motion: |Parameters \(.*\): )$')


class Player:
    """
    run.sh as a subprocess with a growing output buffer.

    Keys go through a pipe, or with tty=True through a pseudo-terminal, as when
    a person types (needed for live control, which reads single key presses).
    """

    def __init__(self, run_sh: Path, *args: str, tty: bool = False):
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        self.master = None
        if tty:
            self.master, slave = pty.openpty()
            self.proc = subprocess.Popen([str(run_sh), *args], stdin=slave, stdout=slave,
                                         stderr=slave, start_new_session=True, env=env)
            os.close(slave)
            os.set_blocking(self.master, False)
        else:
            self.proc = subprocess.Popen([str(run_sh), *args], stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         start_new_session=True, env=env)
            os.set_blocking(self.proc.stdout.fileno(), False)
        self.text, self.mark = '', 0
        STARTED.append(self)

    def read(self) -> None:
        try:
            if self.master is not None:
                chunk = os.read(self.master, 65536)
            else:
                chunk = self.proc.stdout.read()
        except (BlockingIOError, ValueError, OSError):
            chunk = None
        if chunk:
            self.text += chunk.decode(errors='replace')

    def key(self, chars: str) -> None:
        """Type characters without Enter (pseudo-terminal only)."""
        os.write(self.master, chars.encode())

    def new(self) -> str:
        """Output since the last send()."""
        return self.text[self.mark:]

    def send(self, keys: str) -> None:
        self.mark = len(self.text)
        if self.master is not None:
            os.write(self.master, f'{keys}\n'.encode())
            return
        self.proc.stdin.write(f'{keys}\n'.encode())
        self.proc.stdin.flush()

    def at_prompt(self) -> bool:
        return bool(PROMPT.search(self.text)) and len(self.text) > self.mark


class Checker(Node):
    def __init__(self):
        super().__init__('check_player')
        self.limits, self.home = load_robot()
        self.latest, self.samples, self.results = None, [], []
        self.stamps = []  # sim time of each sample
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.arm_a = self.create_publisher(JointTrajectory, '/arm_a_controller/joint_trajectory',
                                           10)
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in m.JOINTS):
            self.latest = np.array([pos[j] for j in m.JOINTS])
            self.samples.append(self.latest)
            self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def report(self, ok, text, context=''):
        """Record a check; on failure also print `context` (the player's output)."""
        ok = bool(ok)
        self.results.append(ok)
        print(f'[{"PASS" if ok else "FAIL"}] {text}', flush=True)
        if not ok and context:
            print('       player output:\n       ' + context[-1500:].replace('\n', '\n       '))
        return ok

    def wait(self, player: Player, timeout: float, until=None) -> bool:
        """Spin and read output until a prompt (or `until(text)`) or timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            player.read()
            if until(player.new()) if until else player.at_prompt():
                return True
            if player.proc.poll() is not None:
                player.read()
                return False
        return False

    def step(self, player, keys, timeout=300.0, until=None):
        """Type `keys`, wait for the next prompt; return (output, joint samples)."""
        self.samples = []
        player.send(keys)
        self.wait(player, timeout, until)
        return player.new(), np.array(self.samples)

    def within_limits(self, label, q):
        lo, hi = self.limits.safe_lower - 0.005, self.limits.safe_upper + 0.005
        lo = np.maximum(np.minimum(lo, q[0]), self.limits.lower)
        hi = np.minimum(np.maximum(hi, q[0]), self.limits.upper)
        return self.report(len(q) and ((q >= lo) & (q <= hi)).all(),
                           f'{label}: joints stayed inside the safe limits')

    def only_moved(self, label, q, joints, at_least):
        """Only `joints` moved (others within 5 mrad), each by >= at_least."""
        idx = [m.INDEX[j] for j in joints]
        span = np.ptp(q, axis=0)
        others = np.delete(span, idx).max()
        self.report(min(span[idx]) >= at_least and others < 0.005,
                    f'{label}: {"+".join(joints)} moved {min(span[idx]):.2f} rad, '
                    f'other joints {others:.4f} rad')

    def at_home_quiet(self) -> bool:
        self.spin_for(0.5)
        return bool(np.abs(self.latest - self.home).max() < 0.02)

    def at_home(self, label):
        self.spin_for(0.5)
        err = np.abs(self.latest - self.home).max()
        return self.report(err < 0.02, f'{label}: robot at home ({err:.4f} rad)')

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def collision_free(self, q) -> bool:
        req = GetStateValidity.Request(group_name='both_arms', robot_state=RobotState())
        req.robot_state.joint_state.name = list(m.JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        future = self.validity.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().valid

    def hold_key(self, player: Player, key: str, seconds: float, settle: float = 2.0):
        """
        Hold `key` like a keyboard: press, auto-repeat after 0.5 s at 33/s, release.

        Returns the joint samples while held and the samples during `settle` s after.
        """
        self.samples, self.stamps = [], []
        end, nxt = time.monotonic() + seconds, time.monotonic() + 0.5
        player.key(key)
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)
            player.read()
            if time.monotonic() >= nxt:
                player.key(key)
                nxt += 1 / 33
        held = np.array(self.samples)
        self.held_stamps = np.array(self.stamps)
        self.samples = []
        self.spin_for(settle)
        player.read()
        return held, np.array(self.samples)

    def box(self, add: bool, x=0.0, y=0.0, z=0.0, size=0.08) -> bool:
        obj = CollisionObject(id='check_player_box')
        obj.header.frame_id = 'world'
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size] * 3)]
        obj.primitive_poses = [Pose()]
        p = obj.primitive_poses[0].position
        p.x, p.y, p.z = x, y, z
        obj.operation = CollisionObject.ADD if add else CollisionObject.REMOVE
        req = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
        req.scene.world.collision_objects = [obj]
        if not self.scene.wait_for_service(timeout_sec=5.0):
            return False
        future = self.scene.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success


def domain_empty(domain: int, timeout=30.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if not sm.processes_on_domain(domain, exclude=(os.getpid(),)):
            return True
        time.sleep(1.0)
    return False


def live_scenario(c: Checker, args, config, domain: int) -> None:
    """Live control (J) through a pseudo-terminal; the transcript is saved to the logs."""
    p = Player(args.run_sh, '--headless', tty=True)
    try:
        _live_scenario(c, p, args, config, domain)
    finally:
        log = config.logs_dir / 'check_player_live.txt'
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text(p.text)
        print(f'player transcript: {log}')
        stop_player(p)


def stop_player(p: Player) -> None:
    """End the player so it shuts its simulation down: Ctrl-C (twice), then SIGTERM."""
    for sig, wait in ((signal.SIGINT, 5), (signal.SIGINT, 60), (signal.SIGTERM, 120),
                      (signal.SIGKILL, 10)):
        if p.proc.poll() is not None:
            return
        os.killpg(p.proc.pid, sig)
        try:
            p.proc.wait(wait)
        except subprocess.TimeoutExpired:
            pass


def still_live(c: Checker, p: Player, what: str) -> bool:
    """Is live control still running (not back at the menu)? Reported only if not."""
    p.read()
    if p.text.rfind('LIVE CONTROL') > p.text.rfind('Choose: '):
        return True
    return c.report(False, f'live control still running before: {what}', p.text[-1500:])


def _live_scenario(c: Checker, p: Player, args, config, domain: int) -> None:
    if not c.report(c.wait(p, config.start_timeout + 30), 'run.sh on a terminal reaches the menu',
                    p.text[-2000:]):
        return
    live = lambda t: 'LIVE CONTROL' in t and '| ready' in t  # noqa: E731
    # from home, declining the lift: joint5 up runs into the floor -> stopped before it
    out, _ = c.step(p, 'j', 30)
    c.report('[Y/n]' in out and 'hit the floor' in out, 'live control at home offers the lift',
             out)
    out, _ = c.step(p, 'n', 30, until=live)
    c.report('T / G  joint5' in out and 'hold a key to move' in out,
             'live control screen lists the keys', out)
    i5 = m.INDEX['arm_a_joint5']
    held, after = c.hold_key(p, 't', 4.0)
    c.report('stopped before a collision' in p.new(), 'live: joint5 up from home stops before '
             'the floor', p.new()[-600:])
    c.report(c.collision_free(c.latest), f'live: robot is collision-free where it stopped '
             f'(joint5 at {np.degrees(c.latest[i5]):+.1f} deg)')
    c.report(len(after) and np.ptp(after, axis=0).max() < 0.005, 'live: holds still after the '
             'collision stop')
    p.key('x')
    c.wait(p, 30)
    c.step(p, 'h', 400.0)
    c.at_home('live, then H')

    # lifted to the DOF pose: hold, release, tap, both directions, the limit, SPACE, exit
    out, _ = c.step(p, 'j', 30)
    out, _ = c.step(p, 'y', 120, until=live)
    c.report('DOF demo pose' in out and 'LIVE CONTROL  Left arm' in out,
             'live control after lifting to the DOF pose', out)
    i3 = m.INDEX['arm_a_joint3']
    if not still_live(c, p, 'E'):
        return
    start = c.latest.copy()
    held, after = c.hold_key(p, 'e', 2.0)
    moved = c.latest - start
    c.only_moved('live hold E (joint3 up)', np.vstack([held, after]), ['arm_a_joint3'], 0.1)
    c.report(moved[i3] > 0.1, f'live: E moves joint3 up ({np.degrees(moved[i3]):+.1f} deg)')
    c.report(len(after) > 20 and np.ptp(after[len(after) // 2:], axis=0).max() < 0.002,
             'live: joint3 stops after the key is released')
    t = c.held_stamps
    if len(held) > 20 and t[-1] > t[0]:
        # speed over 0.2 s windows of sim time
        k = max(1, int(np.searchsorted(t, t[0] + 0.2)))
        speed = float(np.max(np.abs(held[k:, i3] - held[:-k, i3]) / (t[k:] - t[:-k])))
        c.report(0.1 < speed < 0.25, f'live: top speed {speed:.2f} rad/s (jog speed 0.2)')
    if not still_live(c, p, 'D'):
        return
    start = c.latest.copy()
    c.hold_key(p, 'd', 2.0)
    c.report(c.latest[i3] - start[i3] < -0.1,
             f'live: D moves joint3 down ({np.degrees(c.latest[i3] - start[i3]):+.1f} deg)')
    i1 = m.INDEX['arm_a_joint1']
    if not still_live(c, p, 'tap Q'):
        return
    start = c.latest.copy()
    c.hold_key(p, 'q', 0.01)
    tap = c.latest[i1] - start[i1]
    c.report(0.0 < tap < 0.12, f'live: one tap of Q moves joint1 a little '
             f'({np.degrees(tap):+.1f} deg)')
    if not still_live(c, p, 'hold Q to the limit'):
        return
    p.key(']]')  # 0.5 rad/s
    held, after = c.hold_key(p, 'q', 7.0)
    top = max(np.max(held[:, i1]) if len(held) else 0, np.max(after[:, i1]) if len(after) else 0)
    c.report(c.limits.safe_upper[i1] - 0.01 <= c.latest[i1] and
             top <= c.limits.safe_upper[i1] + 0.003,
             f'live: holding Q stops at the safe limit (joint1 '
             f'{np.degrees(c.limits.upper[i1] - top):.2f} deg below the URDF limit)')
    c.report('end of the safe range' in p.new(), 'live: says it is at the end of the range',
             p.new()[-400:])
    p.key('[[')  # back to 0.2 rad/s
    # SPACE: hold A (joint1 down), then SPACE while it moves
    if not still_live(c, p, 'SPACE'):
        return
    c.samples = []
    p.key('a')
    end = time.monotonic() + 1.5
    nxt = time.monotonic() + 0.5
    while time.monotonic() < end:
        rclpy.spin_once(c, timeout_sec=0.005)
        if time.monotonic() >= nxt:
            p.key('a')
            nxt += 1 / 33
    p.key(' ')
    c.spin_for(0.5)
    before = c.latest.copy()
    c.spin_for(2.0)
    p.read()
    c.report('stopped (SPACE)' in p.new() and np.abs(c.latest - before).max() < 0.005,
             'live: SPACE stops and the arm holds', p.new()[-400:])
    p.key('\x1b')
    ok = c.wait(p, 30)
    c.report(ok and 'Done: left live control' in p.new(), 'live: Esc goes back to the menu',
             p.new()[-600:])
    out, _ = c.step(p, 'h', 400.0)
    c.at_home('after live control')
    out, _ = c.step(p, 'q', 400.0, until=lambda t: 'Bye.' in t)
    c.report(p.proc.wait(60) == 0, 'quit after live control exits with code 0')
    c.report(domain_empty(domain), 'no simulation processes left after live control')


def moved_joints(qs, threshold=0.05):
    """Names of the joints that moved more than `threshold` rad in the samples."""
    if not len(qs):
        return set()
    span = np.ptp(qs, axis=0)
    return {j for j, s in zip(m.JOINTS, span) if s > threshold}


def library_scenario(c: Checker, p: Player) -> None:
    arm_a = set(m.ARM_A) | {'arm_a_gripper_joint'}
    arm_b = set(m.ARM_B) | {'arm_b_gripper_joint'}
    c.step(p, 'v', 30)
    c.step(p, '1.0', 30)  # full speed: the gestures were checked at 1.0 before
    c.step(p, '1', 30)    # left arm
    out, _ = c.step(p, 'm', 30)
    c.report('[1] 67 dance' in out and '[2] Dual-arm routine' in out and '[3] Wave' in out
             and 'tempo=54.0 bpm (10..200)' in out, 'M lists the YAML motion library', out)
    out, _ = c.step(p, '3', 30)
    c.report('Parameters (Enter = repetitions=3 arm=arm_a)' in out,
             'wave asks for its parameters with the defaults', out)
    out, qs = c.step(p, '', 300)
    c.report('All checks passed' in out and '> Wave (repetitions=3 arm=arm_a)' in out and
             'Done: back at home' in out, 'left arm: wave played from the library', out)
    c.report(moved_joints(qs) and moved_joints(qs) <= arm_a,
             f'left wave moved only arm_a: {sorted(moved_joints(qs))}')
    c.within_limits('wave', qs)
    c.at_home('after wave')

    c.step(p, '2', 30)  # right arm
    c.step(p, 'm', 30)
    c.step(p, '3', 30)
    out, qs = c.step(p, 'repetitions=1', 300)
    c.report('> Wave (repetitions=1 arm=arm_b)' in out and 'Done: back at home' in out,
             'right arm: wave with repetitions=1 uses arm_b', out)
    c.report(moved_joints(qs) and moved_joints(qs) <= arm_b,
             f'right wave moved only arm_b: {sorted(moved_joints(qs))}')

    c.step(p, 'm', 30)
    c.step(p, '1', 30)
    out, qs = c.step(p, 'tempo=500', 60)
    c.report('Not played: dance67 tempo' in out and not moved_joints(qs, 0.005),
             'out-of-range parameter refused, nothing moved', out)
    out, qs = c.step(p, 'm', 30)
    out, qs = c.step(p, 'moonwalk', 30)
    c.report('"moonwalk" is not in the list' in out and not moved_joints(qs, 0.005),
             'unknown motion refused, nothing moved', out)

    c.step(p, '1', 30)
    c.step(p, 'm', 30)
    c.step(p, 'dual_arm', 30)
    out, qs = c.step(p, '', 600)
    c.report('> Dual-arm routine (repetitions=2)' in out and 'Done: back at home' in out,
             'dual_arm played from the library', out)
    c.within_limits('dual_arm', qs)
    c.at_home('after dual_arm')
    c.step(p, 'm', 30)
    c.step(p, '1', 30)
    out, qs = c.step(p, 'repetitions=2', 600)
    c.report('> 67 dance (repetitions=2 tempo=54.0 amplitude=0.25)' in out and
             'Done: back at home' in out, 'dance67 with repetitions=2', out)
    c.within_limits('dance67', qs)
    c.at_home('after dance67')

    # S during a motion: it stops and the robot holds away from home
    c.step(p, 'm', 30)
    c.step(p, '1', 30)
    p.send('')
    c.samples = []
    c.wait(p, 120.0, until=lambda t: '> 67 dance' in t and 'start (' in t)
    c.spin_for(3.0)
    out, qs = c.step(p, 's')
    c.report('Stopped: ' in out and 'press H' in out, 'S stops a library motion', out)
    c.report(not c.at_home_quiet(), 'robot left away from home by the stop')
    # a motion from away from home: the player returns home first
    c.step(p, 'm', 30)
    c.step(p, '2', 30)
    out, qs = c.step(p, 'repetitions=1', 400)
    c.report('returning home first' in out and 'Done: at home' in out and
             '> Dual-arm routine (repetitions=1)' in out and 'Done: back at home' in out,
             'motion away from home goes home first, then plays', out)
    c.at_home('after dual_arm from away')
    c.step(p, 'v', 30)
    c.step(p, '0.5', 30)


def scenarios(c: Checker, args, config, domain: int) -> None:
    """Run every scenario in order; returns early if the player never starts."""
    # 1. no simulation: --attach must give up with a clear message
    p = Player(args.run_sh, '--attach')
    c.wait(p, 60.0, until=lambda t: 'Bye.' in t)
    c.report(p.proc.wait(10) == 1 and 'still waiting for' in p.text
             and 'arm_a_controller' in p.text,
             'no simulation: --attach reports the missing controllers and exits 1')

    # 2. start headless and reach the menu
    t0 = time.monotonic()
    p = Player(args.run_sh, '--headless')
    ok = c.wait(p, config.start_timeout + 30)
    c.report(ok and 'Simulation ready.' in p.text,
             f'run.sh --headless reaches the menu ({time.monotonic() - t0:.0f} s)')
    if not ok:
        print(p.text[-3000:])
        return
    c.at_home('start')

    # 3. a second instance must refuse the busy domain
    q = Player(args.run_sh, '--headless')
    c.wait(q, 60.0, until=lambda t: 'Bye.' in t or 'already in use' in t)
    c.report(q.proc.wait(10) == 2 and 'already in use' in q.text,
             'second run.sh refuses the busy domain (exit 2)')

    # 4. one joint, left arm: lifts to the DOF pose, then moves only that joint
    out, _ = c.step(p, '1', 30)
    out, qs = c.step(p, 'd', 30)
    c.report('Joints of the left arm' in out and 'range (deg)' in out,
             'DOF menu lists the joints with their ranges')
    out, qs = c.step(p, '1')
    c.report('Moving to the DOF demo pose' in out and out.count('Done') == 2,
             'left joint1: DOF pose + joint1 demo completed', out)
    c.within_limits('left joint1', qs)
    out, qs = c.step(p, 'd', 30)
    out, qs = c.step(p, '4')
    c.report('Done' in out and 'DOF demo pose' not in out,
             'left joint4 from the DOF pose (no extra lift)', out)
    c.only_moved('left joint4', qs, ['arm_a_joint4'], 0.9)

    # 5. right arm and both arms
    out, _ = c.step(p, '2', 30)
    out, _ = c.step(p, 'd', 30)
    out, qs = c.step(p, '5')
    c.only_moved('right joint5', qs, ['arm_b_joint5'], 0.9)
    out, _ = c.step(p, '3', 30)
    out, _ = c.step(p, 'd', 30)
    out, qs = c.step(p, '2')
    c.only_moved('both joint2', qs, ['arm_a_joint2', 'arm_b_joint2'], 0.9)
    out, _ = c.step(p, 'd', 30)
    out, qs = c.step(p, '7')
    c.only_moved('both gripper', qs, ['arm_a_gripper_joint', 'arm_b_gripper_joint'], 0.7)

    # 6. S + Enter stops a motion; the arm holds; H brings it home
    out, _ = c.step(p, '1', 30)
    out, _ = c.step(p, 'd', 30)
    p.send('6')
    c.samples = []
    c.wait(p, 60.0, until=lambda t: 'start (' in t)
    c.spin_for(2.0)
    out, qs = c.step(p, 's')
    c.report('Stopped: ' in out and 'press H' in out, 'S + Enter stops the motion', out)
    before = np.array(c.latest)
    c.spin_for(3.0)
    c.report(np.abs(c.latest - before).max() < 0.01, 'after S the arm holds still')
    out, qs = c.step(p, 'h')
    c.report('Done: at home' in out, 'H returns home', out)
    c.within_limits('home after stop', qs)
    c.at_home('after stop + H')

    # 7. another controller command during a motion is detected
    out, _ = c.step(p, 'd', 30)
    p.send('3')
    c.wait(p, 120.0, until=lambda t: 'DOF demo pose' in t and 'start (' in
           t.split('Done', 1)[-1])
    c.spin_for(1.5)
    hold = JointTrajectoryPoint(positions=[float(c.latest[m.INDEX[j]]) for j in m.ARM_A],
                                velocities=[0.0] * 6)
    hold.time_from_start.nanosec = 500_000_000
    c.arm_a.publish(JointTrajectory(joint_names=m.ARM_A, points=[hold]))
    c.wait(p, 120.0)
    c.report('Motion failed and was stopped' in p.new() and 'off the plan' in p.new(),
             'conflicting controller command is detected and the arm stopped', p.new())

    # 8. home around an obstacle on the direct path: MoveIt plans instead. Start from
    # the DOF pose exactly (home, then one joint demo, which ends back at that pose),
    # because the box is placed on the way home from there.
    c.step(p, 'h', 400.0)
    c.step(p, 'd', 30)
    c.step(p, '1')
    hand = (0.42, 0.14, 0.21)  # arm_a's wrist halfway home from the DOF pose (FK)
    c.report(c.box(True, *hand), 'obstacle placed on the direct way home')
    out, qs = c.step(p, 'h', 400.0)
    c.report('direct path blocked' in out and 'MoveIt plan' in out,
             'home with the direct path blocked uses a MoveIt plan', out)
    c.at_home('home via MoveIt')
    c.report(c.box(False), 'obstacle removed')

    # 9. the removed actions (all joints, step jog, custom sequence) are not options
    for key in 'atc':
        out, qs = c.step(p, key, 30)
        c.report(f'"{key}" is not an option' in out and not moved_joints(qs, 0.005),
                 f'"{key}" is no longer a menu option', out)

    # 9a. live control needs a terminal: through this pipe it is refused, nothing moves
    out, qs = c.step(p, 'j', 30)
    c.report('needs a real terminal' in out and
             (not len(qs) or np.ptp(qs, axis=0).max() < 0.005),
             'live control through a pipe is refused with a hint', out)

    # 10. the motion library: one motion, the selected arm, parameters, refusals,
    # a stop mid-motion, and starting away from home
    library_scenario(c, p)

    # 11. quit from a non-home pose: offers home, then shuts everything down
    out, _ = c.step(p, 'd', 30)
    out, _ = c.step(p, '1')
    out, _ = c.step(p, 'q', 60.0, until=lambda t: '[Y/n]' in t)
    c.report('not at home' in out, 'quit away from home asks to return home', out)
    p.send('y')
    c.wait(p, 400.0, until=lambda t: 'Bye.' in t)
    c.at_home('quit')
    c.report(p.proc.wait(60) == 0 and 'Shutting down' in p.text, 'quit exits with code 0')
    c.report(domain_empty(domain), 'no simulation processes left after quit')

    # 12. Ctrl-C at the menu also shuts down cleanly
    p = Player(args.run_sh, '--headless')
    if c.wait(p, config.start_timeout + 30):
        os.killpg(p.proc.pid, signal.SIGINT)
        c.wait(p, 120.0, until=lambda t: 'Bye.' in t)
        c.report(p.proc.wait(60) == 0, 'Ctrl-C at the menu quits cleanly')
        c.report(domain_empty(domain), 'no simulation processes left after Ctrl-C')
    else:
        c.report(False, 'second start reached the menu')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run-sh', type=Path, default=None,
                        help='run.sh to test (default: the installed one)')
    parser.add_argument('--only', choices=['menu', 'live'],
                        help='run only the menu scenarios or only live control')
    args = parser.parse_args()
    if args.run_sh is None:
        from ament_index_python.packages import get_package_share_directory
        args.run_sh = Path(get_package_share_directory('arm_motion_player')) / 'run.sh'
    config = load_config(args.run_sh.parent / 'config' / 'player.yaml', Path.cwd())
    domain = config.ros_domain_id
    sm.apply_environment(sm.ros_environment(domain, config.gz_partition))
    if not domain_empty(domain, timeout=1.0):
        sys.exit(f'Domain {domain} is in use; stop that first.')

    rclpy.init()
    c = Checker()
    try:
        if args.only != 'live':
            scenarios(c, args, config, domain)
        if args.only != 'menu':
            live_scenario(c, args, config, domain)
    finally:
        for p in STARTED:
            stop_player(p)
        passed, total = sum(c.results), len(c.results)
        print(f'\n{passed}/{total} checks passed', flush=True)
        c.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if total and passed == total else 1)


if __name__ == '__main__':
    main()
