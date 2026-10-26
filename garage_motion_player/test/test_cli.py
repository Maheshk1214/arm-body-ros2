"""Menu behaviour, driven with scripted input and a fake robot (no ROS)."""
from pathlib import Path

from arm_motion_demos import motions as m
from arm_motion_demos.executor import Aborted, Stopped
from garage_motion_player import live_jog
from garage_motion_player.cli import App, HangUp, StdinStopper, Terminal
from garage_motion_player.config import load_config
import numpy as np
import pytest

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'player.yaml'


class FakeRobot:
    """Records calls; `fail` maps a joint name to the exception its DOF demo raises."""

    def __init__(self, home=True, fail=None):
        self.home = np.zeros(len(m.JOINTS))
        self.limits = m.Limits(np.full(14, -3.0), np.full(14, 3.0), np.ones(14), np.ones(14))
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

    def jog(self, joints, speed, delta=None, goal=None, should_stop=None, on_sample=None):
        self.calls.append(('jog', tuple(joints), None if delta is None else round(delta, 6),
                           goal, speed))
        for j in joints:
            if j in self.fail:
                raise self.fail[j]
        self._home = False
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


def run(config, robot, keys, should_stop=lambda: None, terminal=None):
    inputs, lines = iter(keys), []

    def ask(prompt):
        lines.append(prompt)
        try:
            return next(inputs)
        except StopIteration:
            raise EOFError
    App(robot, config, ask=ask, out=lines.append, should_stop=should_stop,
        terminal=terminal).run()
    return '\n'.join(lines)


def calls(robot, kind):
    return [c for c in robot.calls if c[0] == kind]


def test_menu_shows_robot_speed_and_pose(config):
    text = run(config, FakeRobot(), ['q'])
    assert 'GIP GARAGE MOTION PLAYER' in text
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


def test_all_joints_run_in_order_then_home(config):
    robot = FakeRobot()
    text = run(config, robot, ['a', 'q'])
    assert [c[1][0] for c in calls(robot, 'dof')] == [
        f'arm_a_{s}' for s in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
                               'gripper_joint')]
    assert calls(robot, 'home') and 'All joints done' in text


def test_all_joints_start_from_the_dof_pose_from_anywhere(config):
    robot = FakeRobot(home=False)  # e.g. stopped mid-motion
    run(config, robot, ['a', 'q'])
    assert robot.calls[0] == ('goto', 'DOF demo pose')


def test_single_joint_from_the_dof_pose_needs_no_lift(config):
    robot = FakeRobot()
    run(config, robot, ['d', '1', 'd', '2', 'q', 'y'])
    assert calls(robot, 'goto') == [('goto', 'DOF demo pose')]  # only before the first


def test_all_joints_stop_at_the_first_failure(config):
    robot = FakeRobot(fail={'arm_a_joint3': Aborted('arm_a_joint3: deviated 0.3 rad')})
    text = run(config, robot, ['a', 'q', 'n'])
    assert [c[1][0] for c in calls(robot, 'dof')] == ['arm_a_joint1', 'arm_a_joint2',
                                                      'arm_a_joint3']
    assert 'Motion failed and was stopped' in text and 'Stopping the series here' in text
    assert not calls(robot, 'home')  # not sent home blindly after a failure


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


def deg(x):
    return round(np.radians(x), 6)


def test_jog_steps_the_selected_joint(config):
    robot = FakeRobot(home=False)
    text = run(config, robot, ['2', 't', '3', '+', '-', '+++', 'b', 'q', 'n'])
    assert 'JOG  Right arm: joint3' in text and 'Step: 5 deg' in text
    assert [(c[1], c[2]) for c in calls(robot, 'jog')] == [
        (('arm_b_joint3',), deg(5)), (('arm_b_joint3',), deg(-5)), (('arm_b_joint3',), deg(15))]


def test_jog_step_size_and_angle(config):
    robot = FakeRobot(home=False)
    run(config, robot, ['3', 't', '1', '>', '>', '>', '-', '<', '<', '<', '<', '<', '+',
                        'g', '30', 'g', 'abc', 'b', 'q', 'n'])
    jogs = calls(robot, 'jog')
    both = ('arm_a_joint1', 'arm_b_joint1')
    assert [c[1:4] for c in jogs] == [(both, deg(-20), None), (both, deg(1), None),
                                      (both, None, np.radians(30))]


def test_jog_from_home_warns_and_p_lifts(config):
    robot = FakeRobot()
    text = run(config, robot, ['t', '2', 'p', '+', 'b', 'q', 'n'])
    assert 'steps that would hit it are refused' in text
    assert robot.calls[0] == ('goto', 'DOF demo pose') and calls(robot, 'jog')


def test_jog_refusal_is_reported_and_jog_continues(config):
    robot = FakeRobot(home=False, fail={'arm_a_joint5': m.MotionError('collision at t=0.2 s')})
    text = run(config, robot, ['t', '5', '+', 'x', 'b', 'q', 'n'])
    assert 'Not moved: collision at t=0.2 s' in text and '"x" is not a jog key' in text
    assert text.count('JOG  Left arm: joint5') == 3  # shown again after each key


def test_ctrl_c_at_the_jog_prompt_goes_back_to_the_menu(config):
    robot = FakeRobot(home=False)
    keys = iter(['t', '1'])

    def ask(prompt):
        if prompt == 'Jog: ':
            raise KeyboardInterrupt
        try:
            return next(keys)
        except StopIteration:
            raise EOFError
    lines = []
    App(robot, config, ask=ask, out=lines.append, should_stop=lambda: None).run()
    assert sum('GIP GARAGE MOTION PLAYER' in t for t in lines) == 2 and not calls(robot, 'jog')


def test_jog_back_from_the_joint_list(config):
    robot = FakeRobot(home=False)
    run(config, robot, ['t', 'b', 'q', 'n'])
    assert not calls(robot, 'jog')


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
