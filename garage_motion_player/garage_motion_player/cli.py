"""
GIP Garage Motion Player: terminal menu.

Started by run.sh. main() sets up the ROS environment, launches (or attaches
to) the simulation, waits until it is ready and runs the menu. App holds the
menu logic; it talks to the robot only through a Robot-like object, so it can
be tested with a fake one.
"""
import argparse
from contextlib import contextmanager
import math
import os
from pathlib import Path
import re
import select
import shutil
import signal
import sys
import termios
import tty
from typing import Callable, Dict, List, Optional

from arm_motion_demos import motions as m
from arm_motion_demos.executor import Aborted, Stopped
from garage_motion_player import live_jog
from garage_motion_player.config import Config, ConfigError, load_config
import numpy as np

ARM_CHOICES = {'1': 'left', '2': 'right', '3': 'both'}
ARM_NAMES = {'left': 'Left arm', 'right': 'Right arm', 'both': 'Both arms'}
JOG_STEPS = (1.0, 2.0, 5.0, 10.0, 20.0)  # deg, chosen with < and >
JOG_DEFAULT_STEP = 2                     # index into JOG_STEPS (5 deg)
LINE = '=' * 46


def render_main(arm: str, speed: float, pose: str, config: Config) -> str:
    prefixes = {'left': config.arms['left'], 'right': config.arms['right'],
                'both': f'{config.arms["left"]} + {config.arms["right"]}'}
    return '\n'.join([
        '', LINE, '          GIP GARAGE MOTION PLAYER', LINE,
        f' Robot: {ARM_NAMES[arm]} ({prefixes[arm]})   Speed: {speed:.2f}   Pose: {pose}',
        '-' * 46,
        ' SELECT ROBOT   [1] Left arm  [2] Right arm  [3] Both arms',
        ' ACTIONS        [D] Move one joint (DOF)',
        '                [A] Move all joints, one after another',
        '                [J] Live control: hold keys to move joints',
        '                [T] Jog one joint by steps (type + Enter)',
        ' OTHER          [H] Return to home',
        '                [V] Change speed',
        '                [Q] Quit',
        ' While the robot moves: type S + Enter (or Ctrl-C) to stop.',
        LINE])


def render_joints(arm: str, config: Config, limits: m.Limits, positions) -> str:
    """Joint list for the DOF menu, with each joint's own range and position."""
    side = 'left' if arm == 'both' else arm
    rows = [f' Joints of the {ARM_NAMES[arm].lower()}:',
            '      joint     what            range (deg)     now (deg)']
    for k, info in enumerate(config.joints, 1):
        i = m.INDEX[config.joint_name(side, info.short)]
        lo, hi = math.degrees(limits.lower[i]), math.degrees(limits.upper[i])
        now = math.degrees(positions[i])
        rows.append(f'  [{k}] {info.short:<9} {info.label:<15} {lo:6.0f} .. {hi:<6.0f}'
                    f'   {now:7.1f}')
    rows.append('  [B] Back')
    return '\n'.join(rows)


def render_jog(arm: str, info, joints: List[str], limits: m.Limits, positions,
               step_deg: float) -> str:
    """Jog screen: where the jogged joint(s) are, their range, the step and the keys."""
    rows = ['', '-' * 46, f' JOG  {ARM_NAMES[arm]}: {info.short} ({info.label})']
    for j in joints:
        i = m.INDEX[j]
        lo, hi = math.degrees(limits.safe_lower[i]), math.degrees(limits.safe_upper[i])
        rows.append(f'   {j:<22} now {math.degrees(positions[i]):+7.1f} deg'
                    f'   range {lo:+.0f} .. {hi:+.0f}')
    rows += [f' Step: {step_deg:g} deg',
             ' [+] / [-]  one step (++ or --- for several)   [<] / [>]  smaller / bigger step',
             ' [G] go to an angle   [P] lift to the DOF demo pose   [B] back to the menu',
             '-' * 46]
    return '\n'.join(rows)


class Terminal:
    """Single key presses (no Enter, no echo) and a status line, for live control."""

    ESCAPES = re.compile(r'\x1b\[[0-9;]*[A-Za-z~]|\x1bO[A-Za-z]')  # arrow keys etc.

    def __init__(self, stream=None, out=None):
        self.stream = stream or sys.stdin
        self.out = out or sys.stdout

    def available(self) -> bool:
        try:
            return self.stream.isatty()
        except (ValueError, OSError):
            return False

    @contextmanager
    def raw(self):
        """Keys arrive one by one; Ctrl-C still interrupts. Restored (and drained) after."""
        fd = self.stream.fileno()
        saved = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            yield
        finally:
            termios.tcflush(fd, termios.TCIFLUSH)  # auto-repeats left over: not menu input
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            self.out.write('\n')
            self.out.flush()

    def keys(self) -> str:
        fd, data = self.stream.fileno(), b''
        while select.select([fd], [], [], 0)[0]:
            chunk = os.read(fd, 1024)
            if not chunk:
                break
            data += chunk
        return self.ESCAPES.sub('', data.decode(errors='ignore'))

    def status(self, text: str) -> None:
        width = shutil.get_terminal_size((80, 24)).columns
        self.out.write('\r' + text[:width - 1] + '\x1b[K')
        self.out.flush()


class StdinStopper:
    """should_stop() for the executor: S + Enter on the terminal stops the motion."""

    def __call__(self) -> Optional[str]:
        try:
            ready = select.select([sys.stdin], [], [], 0)[0]
        except (ValueError, OSError):  # stdin closed
            return None
        if ready:
            line = sys.stdin.readline()
            if line.strip().lower() in ('s', 'stop'):
                return 'stopped by the user (S)'
        return None


class App:
    def __init__(self, robot, config: Config, ask: Callable[[str], str] = input,
                 out: Callable[[str], None] = print, should_stop=None, terminal=None):
        self.robot, self.config = robot, config
        self.terminal = terminal
        self.ask, self.out = ask, out
        self.should_stop = should_stop or StdinStopper()
        self.arm = 'left'
        self.speed = config.speed

    # --- helpers -----------------------------------------------------------------
    def joints_for(self, short: str) -> List[str]:
        arms = ['left', 'right'] if self.arm == 'both' else [self.arm]
        return [self.config.joint_name(a, short) for a in arms]

    def pose_text(self) -> str:
        try:
            return 'at home' if self.robot.at_home() else 'not at home'
        except m.MotionError:
            return 'unknown'

    def attempt(self, what: str, action: Callable[[], Optional[str]]) -> bool:
        """Run one robot action and report the outcome; True if it completed."""
        self.out(f'\n> {what}')
        try:
            result = action()
        except Stopped as e:
            self.out(f'Stopped: {e}. The robot holds where it is; press H to return home.')
        except Aborted as e:
            self.out(f'Motion failed and was stopped: {e}. Press H to return home.')
        except live_jog.JogInterrupted as e:
            self.out(f'Live control stopped: {e}. The robot holds; press H to return home.')
        except m.MotionError as e:
            self.out(f'Not moved: {e}')
        except HangUp:  # terminal closed / SIGTERM: hold and shut down, nobody to ask
            self.robot.stop('player ending')
            raise
        except KeyboardInterrupt:
            self.robot.stop('interrupted (Ctrl-C)')
            self.out('Stopped (Ctrl-C). The robot holds where it is; press H to return home.')
        else:
            self.out(f'Done{": " + result if result else "."}')
            return True
        return False

    # --- actions -----------------------------------------------------------------
    def dof_pose(self) -> np.ndarray:
        """Full 14-joint DOF demo pose (both arms) from the config."""
        q = self.robot.home.copy()
        for arm in ('left', 'right'):
            for short, value in self.config.dof_pose.items():
                q[m.INDEX[self.config.joint_name(arm, short)]] = value
        return q

    def ensure_dof_pose(self, from_anywhere: bool = False) -> bool:
        """
        Lift the arms to the DOF demo pose first; False if that failed.

        A single joint starts from the current pose unless the robot is at home
        (where several swings would hit the floor); a series always starts from
        the DOF pose (from_anywhere).
        """
        if self.robot.near(self.dof_pose()):
            return True
        if not from_anywhere and self.pose_text() != 'at home':
            return True
        return self.attempt('Moving to the DOF demo pose (arms lifted clear of the floor)',
                            lambda: self.robot.goto('DOF demo pose', self.dof_pose(),
                                                    self.speed, should_stop=self.should_stop))

    def dof(self, short: str) -> Optional[str]:
        info = {j.short: j for j in self.config.joints}[short]
        notes = self.robot.dof(self.joints_for(short), info.amplitude, self.speed,
                               should_stop=self.should_stop)
        return '; '.join(notes) or None

    def pick_joint(self):
        """Show the joint list and return the chosen JointInfo, or None."""
        self.out(render_joints(self.arm, self.config, self.robot.limits,
                               self.robot.positions()))
        choice = self.ask('Joint: ').strip().lower()
        if choice == 'b' or not choice:
            return None
        if not choice.isdigit() or not 1 <= int(choice) <= len(self.config.joints):
            self.out(f'"{choice}" is not in the list.')
            return None
        return self.config.joints[int(choice) - 1]

    def choose_joint(self) -> None:
        info = self.pick_joint()
        if info is None:
            return
        if not self.ensure_dof_pose():
            return
        self.attempt(f'{ARM_NAMES[self.arm]}: {info.short} ({info.label})',
                     lambda: self.dof(info.short))

    def all_joints(self) -> None:
        """Every joint in turn from the DOF demo pose, then home; stops at a failure."""
        if not self.ensure_dof_pose(from_anywhere=True):
            return
        for info in self.config.joints:
            if not self.attempt(f'{ARM_NAMES[self.arm]}: {info.short} ({info.label})',
                                lambda: self.dof(info.short)):
                self.out('Stopping the series here.')
                return
        self.out('All joints done.')
        self.home()

    def live_names(self) -> Dict[str, List[str]]:
        """Short joint name -> full name(s) on the selected arm(s), for the live keys."""
        shorts = [j.short for j in self.config.joints][:len(live_jog.KEY_PAIRS)]
        return {s: self.joints_for(s) for s in shorts}

    def live_screen(self, names: Dict[str, List[str]]) -> str:
        q, lim = self.robot.positions(), self.robot.limits
        rows = []
        for (up, down), info in zip(live_jog.KEY_PAIRS, self.config.joints):
            i = m.INDEX[names[info.short][0]]
            rows.append((up.upper(), down.upper(), info.short, info.label,
                         math.degrees(q[i]), math.degrees(lim.safe_lower[i]),
                         math.degrees(lim.safe_upper[i])))
        return live_jog.render(ARM_NAMES[self.arm], rows,
                               live_jog.SPEEDS[live_jog.DEFAULT_SPEED], 'ready')

    def live_line(self, names: Dict[str, List[str]], status: str) -> str:
        """Status line: every joint's angle (deg) on the first selected arm, then status."""
        state = self.robot.state()
        if state is None:
            return status
        angles = ' '.join(f'{up.upper()}{math.degrees(state[1][m.INDEX[js[0]]]):+.0f}'
                          for (up, _), js in zip(live_jog.KEY_PAIRS, names.values()))
        return f' {angles} | {status}'

    def live(self) -> None:
        """Hold a key to move a joint, let go to stop (see live_jog)."""
        if self.terminal is None or not self.terminal.available():
            self.out('Live control reads single key presses, so it needs a real terminal. '
                     'Use T to jog by steps instead.')
            return
        if self.pose_text() == 'at home':
            answer = self.ask('At home some directions would hit the floor. Lift the arms to '
                              'the DOF demo pose first? [Y/n] ')
            if answer.strip().lower() not in ('n', 'no') and \
                    not self.ensure_dof_pose(from_anywhere=True):
                return
        names = self.live_names()
        self.out(self.live_screen(names))

        def session():
            with self.terminal.raw():
                return self.robot.live(
                    names, self.terminal.keys,
                    lambda status: self.terminal.status(self.live_line(names, status)),
                    self.config.first_gap, self.config.repeat_gap)
        self.attempt('Live control', session)

    def jog(self) -> None:
        """
        Step one joint (or the same joint on both arms) with + / -, or to a typed angle.

        Each step is its own checked motion: it stays in the safe band and is
        collision-checked by MoveIt, so near the floor some steps are refused.
        """
        info = self.pick_joint()
        if info is None:
            return
        joints = self.joints_for(info.short)
        step = JOG_DEFAULT_STEP
        if self.pose_text() == 'at home':
            self.out(' At home the grippers hang ~2 cm above the floor: steps that would hit it '
                     'are refused.\n Press P to lift the arms to the DOF demo pose first.')
        while True:
            self.out(render_jog(self.arm, info, joints, self.robot.limits,
                                self.robot.positions(), JOG_STEPS[step]))
            try:
                key = self.ask('Jog: ').strip().lower()
            except KeyboardInterrupt:
                return
            if key in ('b', 'q'):
                return
            if not key or key == 's':  # nothing moving: just show the screen again
                continue
            if set(key) == {'+'} or set(key) == {'-'} or key == '=':
                n = len(key) if key != '=' else 1  # '=' is '+' without Shift
                delta = math.radians(JOG_STEPS[step]) * n * (1 if key[0] in '+=' else -1)
                self.attempt(f'Jog {info.short} {math.degrees(delta):+g} deg',
                             lambda: self._jog(joints, delta=delta))
            elif key in ('<', ',', '>', '.'):
                step = max(0, step - 1) if key in '<,' else min(len(JOG_STEPS) - 1, step + 1)
            elif key == 'g':
                text = self.ask('Angle in degrees: ').strip()
                try:
                    goal = math.radians(float(text))
                except ValueError:
                    self.out(f'"{text}" is not a number.')
                    continue
                self.attempt(f'Jog {info.short} to {float(text):g} deg',
                             lambda: self._jog(joints, goal=goal))
            elif key == 'p':
                self.ensure_dof_pose(from_anywhere=True)
            else:
                self.out(f'"{key}" is not a jog key.')

    def _jog(self, joints: List[str], delta=None, goal=None) -> Optional[str]:
        notes = self.robot.jog(joints, self.speed, delta=delta, goal=goal,
                               should_stop=self.should_stop)
        return '; '.join(notes) or None

    def change_speed(self) -> None:
        text = self.ask(f'Speed 0.1 (slow) .. 1.0 (fastest) [now {self.speed:.2f}]: ').strip()
        try:
            value = float(text)
        except ValueError:
            self.out(f'"{text}" is not a number; speed unchanged.')
            return
        if not 0.1 <= value <= 1.0:
            self.out('Speed must be between 0.1 and 1.0; unchanged.')
            return
        self.speed = value

    def home(self) -> None:
        self.attempt('Return to home',
                     lambda: self.robot.go_home(self.speed, should_stop=self.should_stop))

    def quit_flow(self) -> None:
        if self.pose_text() == 'not at home':
            try:
                answer = self.ask('The robot is not at home. Return home before quitting? [Y/n] ')
            except HangUp:
                raise
            except (EOFError, KeyboardInterrupt):  # no answer possible: leave it where it is
                self.out('\nQuitting without returning home.')
                return
            if answer.strip().lower() not in ('n', 'no'):
                self.home()

    # --- loop --------------------------------------------------------------------
    def run(self) -> None:
        actions = {'d': self.choose_joint, 'a': self.all_joints, 'j': self.live, 't': self.jog,
                   'h': self.home, 'v': self.change_speed}
        while True:
            self.out(render_main(self.arm, self.speed, self.pose_text(), self.config))
            try:
                choice = self.ask('Choose: ').strip().lower()
            except HangUp:
                raise
            except (EOFError, KeyboardInterrupt):
                choice = 'q'
            if choice in ARM_CHOICES:
                self.arm = ARM_CHOICES[choice]
            elif choice in actions:
                try:
                    actions[choice]()
                except EOFError:
                    choice = 'q'
            elif choice != 'q':
                self.out(f'"{choice}" is not an option.')
            if choice == 'q':
                self.quit_flow()
                return


def find_world() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('arm_motion_demos')) / 'worlds' / \
        'demo_recording.sdf'


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='GIP Garage Motion Player')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--attach', action='store_true',
                        help='use a simulation that is already running on the configured domain')
    parser.add_argument('--headless', action='store_true', help='no Gazebo window')
    args = parser.parse_args(argv)

    try:
        config = load_config(args.config, args.workspace)
    except ConfigError as e:
        print(f'Configuration error: {e}')
        return 2

    from garage_motion_player import simulation_manager as sm
    sm.apply_environment(sm.ros_environment(config.ros_domain_id, config.gz_partition))
    sim = None
    if not args.attach:
        busy = sm.processes_on_domain(config.ros_domain_id, exclude=(os.getpid(), os.getppid()))
        if busy:
            print(f'ROS domain {config.ros_domain_id} is already in use by:')
            for pid, cmd in busy[:5]:
                print(f'   {pid}  {cmd}')
            print('Close that simulation first, use ./run.sh --attach to use it, or change '
                  'ros_domain_id in config/player.yaml.')
            return 2
        sim = sm.Simulation(find_world(), config.gui and not args.headless, config.rviz,
                            config.logs_dir)

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from garage_motion_player.robot import Robot
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # Ctrl-C = KeyboardInterrupt
    for sig in (signal.SIGHUP, signal.SIGTERM):  # terminal closed / kill: still shut down the sim
        signal.signal(sig, _hang_up)
    node = Node('garage_motion_player')
    code = 0
    try:
        robot = Robot(node, print, config.tracking_tolerance)
        if sim:
            print(f'Starting the simulation (log: {sim.log_path}) ...')
            sim.start()
        sm.wait_ready(robot.missing, config.start_timeout if sim else 30.0,
                      progress=lambda t: print(f'  ... {t}'),
                      poll=lambda: rclpy.spin_once(node, timeout_sec=0.1), sim=sim)
        print('Simulation ready.')
        App(robot, config, terminal=Terminal()).run()
    except (sm.SimulationError, m.MotionError) as e:
        print(f'Error: {e}')
        code = 1
    except KeyboardInterrupt:
        say('\nInterrupted.')
        code = 130
    except OSError as e:  # e.g. the terminal went away mid-print
        code = 1
        say(f'Error: {e}')
    finally:
        if sim:
            say('Shutting down the simulation ...')
            sim.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    say('Bye.')
    return code


class HangUp(KeyboardInterrupt):
    """The terminal was closed (SIGHUP) or the player was told to end (SIGTERM)."""


def _hang_up(signum, frame):
    for sig in (signal.SIGHUP, signal.SIGTERM):  # once: do not interrupt the shutdown
        signal.signal(sig, signal.SIG_IGN)
    raise HangUp


def say(text: str) -> None:
    """Print, unless the terminal is gone (closed window): shutting down matters more."""
    try:
        print(text, flush=True)
    except OSError:
        pass


if __name__ == '__main__':
    sys.exit(main())
