"""Menu behaviour, driven with scripted input and a fake robot (no ROS)."""
from pathlib import Path

from arm_motion_demos import motion_library as ml
from arm_motion_demos import motions as m
from arm_motion_demos.executor import Aborted, Stopped
from arm_motion_player import live_jog
from arm_motion_player.cli import App, HangUp, StdinStopper, Terminal
from arm_motion_player.config import load_config
import numpy as np
import pytest

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'player.yaml'


class FakeRobot:
    """Records calls; `fail` maps a joint name to the exception its DOF demo raises."""

    def __init__(self, home=True, fail=None):
        self.home = np.zeros(len(m.JOINTS))
        # velocity / acceleration as on the robot (the motion checks use them)
        self.limits = m.Limits(np.full(14, -3.0), np.full(14, 3.0), np.full(14, 2.0),
                               np.full(14, 3.0))
        self._home = home
        self._dof_pose = False
        self.fail = fail or {}
        self.calls = []

    def at_home(self):
        return self._home

    def near(self, target):
        return self._dof_pose

    def positions(self):
        return np.zeros(len(m.JOINTS))

    def dof(self, joints, amplitude, speed, should_stop=None, on_sample=None):
        self.calls.append(('dof', tuple(joints), amplitude, speed))
        for j in joints:
            if j in self.fail:
                raise self.fail[j]
        if should_stop and should_stop():
            raise Stopped(f'{joints[0]}: {should_stop()}')
        return []

    def state(self):
        return (0.0, self.positions())

    def live(self, names, keys, show, first_gap, repeat_gap):
        self.calls.append(('live', dict(names), first_gap, repeat_gap))
        show('moving arm_a_joint3 +')
        if 'live' in self.fail:
            raise self.fail['live']
        return 'left live control (robot within 0.010 rad of the command)'

    def goto(self, name, target, speed, should_stop=None, on_sample=None):
        self.calls.append(('goto', name))
        self._home, self._dof_pose = False, True

    def go_home(self, speed, should_stop=None, on_sample=None):
        self.calls.append(('home', speed))
        self._home, self._dof_pose = True, False
        return 'at home (direct path)'

    def stop(self, reason):
        self.calls.append(('stop', reason))

    def check_motion(self, name, motion):
        self.calls.append(('check', name))
        if ('check', name) in self.fail:
            raise self.fail[('check', name)]

    def play(self, name, motion, collision_check=True, should_stop=None, on_sample=None):
        self.calls.append(('play', name, motion))
        if ('play', name) in self.fail:
            raise self.fail[('play', name)]
        self._home = True
        return 0.012


@pytest.fixture
def config(tmp_path):
    return load_config(CONFIG, tmp_path)


class FakeTerminal:
    def __init__(self, tty=True):
        self.tty, self.raw_entered, self.lines = tty, 0, []

    def available(self):
        return self.tty

    def raw(self):
        terminal = self

        class Raw:
            def __enter__(self):
                terminal.raw_entered += 1

            def __exit__(self, *exc):
                return False
        return Raw()

    def keys(self):
        return ''

    def status(self, text):
        self.lines.append(text)


LIBRARY = ml.load_library([ml.builtin_folder()])[0]


def run(config, robot, keys, should_stop=lambda: None, terminal=None, library=LIBRARY):
    inputs, lines = iter(keys), []

    def ask(prompt):
        lines.append(prompt)
        try:
            return next(inputs)
        except StopIteration:
            raise EOFError
    App(robot, config, ask=ask, out=lines.append, should_stop=should_stop,
        terminal=terminal, library=library).run()
    return '\n'.join(lines)


def calls(robot, kind):
    return [c for c in robot.calls if c[0] == kind]


def test_menu_shows_robot_speed_and_pose(config):
    text = run(config, FakeRobot(), ['q'])
    assert 'MORETEA ARM MOTION PLAYER' in text
    assert 'Robot: Left arm (arm_a)' in text and 'Speed: 0.50' in text and 'at home' in text


def test_single_joint_on_the_selected_arm(config):
    robot = FakeRobot()
    run(config, robot, ['2', 'd', '3', 'q', 'n'])
    assert calls(robot, 'goto') == [('goto', 'DOF demo pose')]  # lifted off the floor first
    assert calls(robot, 'dof') == [('dof', ('arm_b_joint3',), 0.5, 0.5)]


def test_both_arms_move_the_same_joint(config):
    robot = FakeRobot(home=False)  # not at home: run from the current pose
    run(config, robot, ['3', 'd', '7', 'q', 'n'])
    assert calls(robot, 'goto') == []
    assert calls(robot, 'dof') == [('dof', ('arm_a_gripper_joint', 'arm_b_gripper_joint'),
                                    0.8, 0.5)]


@pytest.mark.parametrize('choice', ['0', '8', 'x', 'joint1'])
def test_invalid_joint_choices_do_nothing(config, choice):
    robot = FakeRobot(home=False)
    text = run(config, robot, ['d', choice, 'q', 'n'])
    assert calls(robot, 'dof') == [] and 'not in the list' in text


def test_unknown_menu_option(config):
    assert '"z" is not an option' in run(config, FakeRobot(), ['z', 'q'])


def test_speed_is_validated(config):
    robot = FakeRobot(home=False)
    text = run(config, robot, ['v', '5', 'v', 'abc', 'v', '0.3', 'd', '1', 'q', 'n'])
    assert 'between 0.1 and 1.0' in text and 'not a number' in text
    assert calls(robot, 'dof')[0][3] == 0.3


def test_single_joint_from_the_dof_pose_needs_no_lift(config):
    robot = FakeRobot()
    run(config, robot, ['d', '1', 'd', '2', 'q', 'y'])
    assert calls(robot, 'goto') == [('goto', 'DOF demo pose')]  # only before the first


def test_failed_motion_is_reported_and_not_sent_home(config):
    robot = FakeRobot(home=False, fail={'arm_a_joint3': Aborted('arm_a_joint3: deviated 0.3 rad')})
    text = run(config, robot, ['d', '3', 'q', 'n'])
    assert 'Motion failed and was stopped: arm_a_joint3: deviated 0.3 rad' in text
    assert 'Press H' in text and not calls(robot, 'home')


def test_refused_motion_is_reported_and_menu_continues(config):
    robot = FakeRobot(home=False, fail={'arm_a_joint5': m.MotionError('every direction '
                                                                      'would collide')})
    text = run(config, robot, ['d', '5', 'd', '1', 'q', 'n'])
    assert 'Not moved: every direction would collide' in text
    assert len(calls(robot, 'dof')) == 2


def test_stop_request_is_reported(config):
    robot = FakeRobot(home=False)
    text = run(config, robot, ['d', '1', 'q', 'n'], should_stop=lambda: 'stopped by the user')
    assert 'Stopped: arm_a_joint1: stopped by the user' in text and 'press H' in text


def test_ctrl_c_during_a_motion_holds_the_robot(config):
    robot = FakeRobot(home=False, fail={'arm_a_joint1': KeyboardInterrupt()})
    text = run(config, robot, ['d', '1', 'q', 'n'])
    assert calls(robot, 'stop') == [('stop', 'interrupted (Ctrl-C)')]
    assert 'Stopped (Ctrl-C)' in text


def test_quit_offers_home_when_not_at_home(config):
    robot = FakeRobot(home=False)
    run(config, robot, ['q', 'y'])
    assert calls(robot, 'home')
    robot = FakeRobot(home=False)
    run(config, robot, ['q', 'n'])
    assert not calls(robot, 'home')


def test_end_of_input_quits_cleanly(config):
    robot = FakeRobot()
    run(config, robot, [])  # EOF at the first prompt
    assert robot.calls == []


def test_stdin_stopper_reads_s(monkeypatch, tmp_path):
    for text, expected in (('s\n', 'stopped by the user (S)'), ('x\n', None)):
        f = open(tmp_path / 'in.txt', 'w+')
        f.write(text)
        f.seek(0)
        monkeypatch.setattr('sys.stdin', f)
        assert StdinStopper()() == expected
        f.close()


def test_end_of_input_away_from_home_quits_without_moving(config):
    robot = FakeRobot(home=False)
    text = run(config, robot, [])
    assert 'Quitting without returning home' in text and robot.calls == []


# --- live control (J) ------------------------------------------------------------------------
def test_live_control_needs_a_terminal(config):
    robot = FakeRobot(home=False)
    text = run(config, robot, ['j', 'q', 'n'], terminal=FakeTerminal(tty=False))
    assert 'needs a real terminal' in text and not calls(robot, 'live')
    text = run(config, robot, ['j', 'q', 'n'])  # no terminal at all
    assert 'needs a real terminal' in text


def test_live_control_runs_in_raw_mode_with_every_joint(config):
    robot, term = FakeRobot(home=False), FakeTerminal()
    text = run(config, robot, ['3', 'j', 'q', 'n'], terminal=term)
    (_, names, first, repeat), = calls(robot, 'live')
    assert names['joint3'] == ['arm_a_joint3', 'arm_b_joint3'] and len(names) == 7
    assert (first, repeat) == (config.first_gap, config.repeat_gap)
    assert term.raw_entered == 1 and 'LIVE CONTROL  Both arms' in text
    assert 'E / D  joint3' in text and 'Done: left live control' in text
    assert term.lines[-1].endswith('| moving arm_a_joint3 +') and ' Q+0 ' in term.lines[-1]


def test_live_control_from_home_offers_to_lift_first(config):
    robot = FakeRobot()
    run(config, robot, ['j', 'y', 'q', 'n'], terminal=FakeTerminal())
    assert [c[0] for c in robot.calls] == ['goto', 'live']
    robot = FakeRobot()
    run(config, robot, ['j', 'n', 'q', 'n'], terminal=FakeTerminal())
    assert [c[0] for c in robot.calls] == ['live']


def test_live_control_problem_is_reported(config):
    robot = FakeRobot(home=False, fail={'live': live_jog.JogInterrupted('no /joint_states')})
    text = run(config, robot, ['j', 'q', 'n'], terminal=FakeTerminal())
    assert 'Live control stopped: no /joint_states' in text and 'press H' in text


def test_terminal_reports_no_tty_for_a_file(tmp_path):
    with open(tmp_path / 'in.txt', 'w+') as f:
        assert not Terminal(stream=f).available()


def test_terminal_reads_single_keys_without_enter_on_a_pty():
    import io
    import os
    import pty
    import termios
    master, slave = pty.openpty()
    stream, out = os.fdopen(slave, 'r'), io.StringIO()
    term = Terminal(stream=stream, out=out)
    try:
        assert term.available()
        before = termios.tcgetattr(slave)
        with term.raw():
            os.write(master, b'e\x1b[Aq')  # e, arrow up (ignored), q: no Enter
            time_left = 50
            keys = ''
            while len(keys) < 2 and time_left:
                keys += term.keys()
                time_left -= 1
                os.sched_yield()
            assert keys == 'eq'
            term.status('hello')
        assert termios.tcgetattr(slave) == before  # line mode restored
        assert out.getvalue().startswith('\rhello')
    finally:
        stream.close()
        os.close(master)


def test_hang_up_during_a_motion_holds_and_ends_without_questions(config):
    robot = FakeRobot(home=False, fail={'arm_a_joint1': HangUp()})
    with pytest.raises(HangUp):
        run(config, robot, ['d', '1', 'q', 'n'])
    assert calls(robot, 'stop') == [('stop', 'player ending')] and not calls(robot, 'home')


def test_hang_up_at_the_menu_ends_without_the_home_question(config):
    robot = FakeRobot(home=False)
    asked = []

    def ask(prompt):
        asked.append(prompt)
        raise HangUp
    with pytest.raises(HangUp):
        App(robot, config, ask=ask, out=lambda t: None).run()
    assert asked == ['Choose: ']


# --- motion library (M) ----------------------------------------------
def played(robot):
    return [c[1] for c in robot.calls if c[0] == 'play']


def test_menu_lists_the_library_and_no_removed_actions(config):
    text = run(config, FakeRobot(), ['q'])
    assert '[M] Play a motion: dance67, dual_arm, wave' in text
    assert '[A]' not in text and '[T]' not in text and '[C]' not in text


def test_play_a_motion_with_defaults(config):
    robot = FakeRobot()
    text = run(config, robot, ['m', '1', '', 'q'])
    assert '[1] 67 dance' in text and 'All checks passed' in text
    assert calls(robot, 'check') == [('check', 'dance67')] and played(robot) == ['dance67']
    assert '> 67 dance (repetitions=6 tempo=54.0 amplitude=0.25)' in text
    assert 'Done: back at home, at most 0.012 rad from the plan' in text


def test_parameters_and_speed_reach_the_motion(config):
    robot = FakeRobot()
    run(config, robot, ['v', '0.3', 'm', 'dance67', 'repetitions=2 tempo=40', 'q'])
    (_, _, motion), = calls(robot, 'play')
    expected = LIBRARY['dance67'].build(robot.home, robot.limits,
                                        {'repetitions': 2, 'tempo': 40.0}, 0.3)
    assert np.array_equal(motion.q, expected.q)


def test_the_selected_arm_waves(config):
    robot = FakeRobot()
    run(config, robot, ['2', 'm', '3', '', 'q'])
    (_, _, motion), = calls(robot, 'play')
    moved = np.ptp(motion.q, axis=0)
    assert moved[m.INDEX['arm_b_joint1']] > 1.0 and moved[m.INDEX['arm_a_joint1']] == 0.0


def test_not_at_home_goes_home_first(config):
    robot = FakeRobot(home=False)
    run(config, robot, ['m', '2', '', 'q'])
    kinds = [c[0] for c in robot.calls]
    assert kinds == ['check', 'home', 'play']  # checked before anything moves


def test_bad_parameters_or_choice_play_nothing(config):
    robot = FakeRobot()
    text = run(config, robot, ['m', '1', 'tempo=500', 'm', '9', 'm', 'b', 'q'])
    assert 'Not played: dance67 tempo' in text and '"9" is not in the list' in text
    assert not played(robot) and not calls(robot, 'check')


def test_a_refused_check_plays_nothing(config):
    robot = FakeRobot(fail={('check', 'wave'): m.MotionError('wave: collision at t=1.20 s')})
    text = run(config, robot, ['m', 'wave', '', 'q'])
    assert 'Not played: wave: collision at t=1.20 s' in text and not played(robot)


def test_empty_library_says_so(config):
    text = run(config, FakeRobot(), ['m', 'q'], library={})
    assert '(none found)' in text and 'No motions found' in text


def test_removed_keys_are_not_options(config):
    robot = FakeRobot()
    text = run(config, robot, ['a', 't', 'c', 'q'])
    assert all(f'"{k}" is not an option' in text for k in 'atc') and robot.calls == []
