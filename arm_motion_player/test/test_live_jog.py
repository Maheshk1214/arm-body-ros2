"""Live control: key hold detection, velocity ramp, limits, collision stop, failures."""
from arm_motion_demos import motions as m
from arm_motion_player import live_jog as lj
import numpy as np
import pytest

RTF = 0.6  # sim seconds per wall second, as in Gazebo here
ARM_A = {s: [f'arm_a_{suffix}'] for s, suffix in [  # as the menu passes them: menu order
    ('joint1', 'joint1'), ('joint2', 'joint2'), ('joint3', 'joint3'), ('joint4', 'joint4'),
    ('joint5', 'joint5'), ('joint6', 'joint6'), ('gripper', 'gripper_joint')]}


def limits():
    lo, hi = np.full(14, -1.0), np.full(14, 1.0)
    return m.Limits(lo, hi, np.full(14, 2.0), np.full(14, 3.0))


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class SimRobot:
    """Follows every streamed reference exactly; optional collision zone and faults."""

    def __init__(self, clock, q=None, collide_above=None, lag=0.0, stale_after=None):
        self.clock, self.limits = clock, limits()
        self.q = np.zeros(14) if q is None else np.array(q, float)
        self.collide_above = collide_above or {}
        self.lag, self.stale_after = lag, stale_after
        self.streams, self.stops, self.history = [], [], []

    def state(self):
        return (self.clock() * RTF, self.q.copy())

    def state_age(self):
        return 99.0 if self.stale_after is not None and self.clock() > self.stale_after else 0.0

    def stream(self, reference, velocity, horizon, joints, stamp):
        assert stamp == pytest.approx(self.clock() * RTF)  # the reference's own time
        self.streams.append((np.array(reference), np.array(velocity)))
        self.q = np.array(reference) - self.lag * np.sign(velocity)
        self.history.append(self.q.copy())

    def collision(self, q):
        for j, edge in self.collide_above.items():
            if q[m.INDEX[j]] > edge:
                return f'{j} hits the floor'
        return None

    def stop(self, reason):
        self.stops.append(reason)


def keys_from(clock, events):
    """Turn [(wall time, chars)] into keys(), which returns what was typed since its last call."""
    events = sorted(events)

    def keys():
        out = ''
        while events and events[0][0] <= clock():
            out += events.pop(0)[1]
        return out
    return keys


def held(key, start, duration, rate=33.0):
    """Hold a key: first press, then auto-repeat after 0.5 s at `rate` per second."""
    times = [start] + list(np.arange(start + 0.5, start + duration, 1.0 / rate))
    return [(t, key) for t in times]


def session(robot, clock, events, names=None, **kw):
    names = names or ARM_A
    shown = []
    jog = lj.LiveJogger(robot, names, keys_from(clock, events), shown.append, clock=clock,
                        sleep=clock.sleep, **kw)
    return jog, shown


def index(j):
    return m.INDEX[j]


# --- KeyHold ------------------------------------------------------------------------------
def test_key_hold_waits_for_the_repeat_delay_then_tracks_repeats():
    k = lj.KeyHold(first_gap=0.6, repeat_gap=0.15)
    k.press('e', 0.0)
    assert k.held(0.55) == 'e'      # still waiting for the first auto-repeat
    k.press('e', 0.5)
    assert k.held(0.6) == 'e'
    assert k.held(0.66) is None     # repeats stopped for > 0.15 s: released
    k.press('e', 1.0)
    assert k.held(1.5) == 'e' and k.held(1.7) is None  # a fresh press: first gap again


def test_another_key_takes_over():
    k = lj.KeyHold()
    k.press('e', 0.0)
    k.press('d', 0.1)
    assert k.held(0.2) == 'd'


# --- Ramp ---------------------------------------------------------------------------------
def test_ramp_accelerates_to_speed_and_brakes_to_rest():
    r = lj.Ramp(0.0, -1.0, 1.0, accel=1.5)
    for _ in range(100):
        r.step(0.01, 0.3)
    assert r.velocity == pytest.approx(0.3)
    for _ in range(100):
        r.step(0.01, 0.0)
    assert r.velocity == 0.0 and not r.moving


def test_ramp_never_passes_the_band_edge():
    r = lj.Ramp(0.9, -1.0, 1.0, accel=1.5)
    peak = 0.0
    for _ in range(500):
        r.step(0.01, 0.5)
        peak = max(peak, r.position)
    assert peak <= 1.0 and r.position == pytest.approx(1.0, abs=1e-3)


def test_braking_point_is_ahead_of_the_motion():
    r = lj.Ramp(0.0, -1.0, 1.0, accel=2.0, velocity=-0.4)
    assert r.braking_point(0.25) == pytest.approx(-(0.16 / 4 + 0.1))


# --- LiveJogger ---------------------------------------------------------------------------
def test_holding_a_key_moves_only_that_joint_and_releasing_stops_it():
    clock, robot = Clock(), None
    robot = SimRobot(clock)
    jog, shown = session(robot, clock, held('e', 0.0, 2.0) + [(4.0, 'x')])
    jog.run()
    moved = np.ptp(np.array(robot.history), axis=0)
    assert moved[index('arm_a_joint3')] > 0.2
    assert np.delete(moved, index('arm_a_joint3')).max() == 0.0
    final = robot.streams[-1]
    assert not final[1].any()        # last command: at rest
    assert any(s.startswith('moving arm_a_joint3 +') for s in shown) and shown[-1] == 'stopped'
    speeds = np.abs(np.diff([h[index('arm_a_joint3')] for h in robot.history])) / (lj.TICK * RTF)
    assert speeds.max() <= lj.SPEEDS[lj.DEFAULT_SPEED] + 1e-6


def test_motion_stops_soon_after_the_key_is_released():
    clock = Clock()
    robot = SimRobot(clock)
    jog, _ = session(robot, clock, held('d', 0.0, 2.0) + [(5.0, 'x')])
    times = []
    jog.sleep = lambda dt: (times.append((clock(), robot.q[index('arm_a_joint3')])),
                            clock.sleep(dt))
    jog.run()
    last_change = max(t for (t, q), (_, q2) in zip(times, times[1:]) if q2 != q)
    assert last_change < 2.0 + 0.15 + 0.4  # repeat gap + braking (0.2 rad/s at 1.5 rad/s^2)
    assert robot.q[index('arm_a_joint3')] < 0  # 'd' is down


def test_a_single_tap_moves_a_little():
    clock = Clock()
    robot = SimRobot(clock)
    jog, _ = session(robot, clock, [(0.0, 'q'), (3.0, 'x')])
    jog.run()
    assert 0.0 < robot.q[index('arm_a_joint1')] < 0.2 * 0.6 * RTF + 0.02


def test_both_arms_move_together():
    clock = Clock()
    robot = SimRobot(clock)
    names = {'joint4': ['arm_a_joint4', 'arm_b_joint4']}
    jog, _ = session(robot, clock, held('q', 0.0, 1.5) + [(3.0, 'x')], names=names)
    jog.run()
    a, b = robot.q[index('arm_a_joint4')], robot.q[index('arm_b_joint4')]
    assert a > 0.1 and a == pytest.approx(b)


def test_holding_into_the_limit_stops_inside_the_safe_band():
    clock = Clock()
    robot = SimRobot(clock, q=np.full(14, 0.8))
    jog, shown = session(robot, clock, held('e', 0.0, 4.0) + [(5.0, 'x')])
    jog.run()
    i = index('arm_a_joint3')
    peak = max(h[i] for h in robot.history)
    assert peak <= robot.limits.safe_upper[i] + 1e-9
    assert robot.q[i] == pytest.approx(robot.limits.safe_upper[i], abs=2e-3)
    assert any('end of the safe range' in s for s in shown)


def test_collision_ahead_stops_and_blocks_that_direction():
    clock = Clock()
    robot = SimRobot(clock, collide_above={'arm_a_joint3': 0.3})
    events = held('e', 0.0, 4.0) + held('e', 5.0, 1.5) + held('d', 7.0, 1.0) + [(9.0, 'x')]
    jog, shown = session(robot, clock, events)
    trace = []
    jog.sleep = lambda dt: (trace.append((clock(), robot.q[index('arm_a_joint3')])),
                            clock.sleep(dt))
    jog.run()
    q3 = [q for _, q in trace]
    assert max(q3) < 0.3  # never commanded into the colliding zone
    assert robot.stops and 'collision ahead' in robot.stops[0]
    assert any('stopped before a collision' in s for s in shown)
    blocked = [q for t, q in trace if 5.0 <= t < 7.0]
    assert np.ptp(blocked) < 1e-9  # 'e' again: refused, nothing moves
    assert q3[-1] < min(blocked) - 0.05  # 'd' (the other way) works


def test_space_stops_at_once_and_speed_keys_change_the_speed():
    clock = Clock()
    robot = SimRobot(clock)
    events = [(0.0, ']]]]]')] + held('e', 0.1, 1.0) + [(1.2, ' '), (3.0, 'x')]
    jog, shown = session(robot, clock, events)
    trace = []
    jog.sleep = lambda dt: (trace.append((clock(), robot.q[index('arm_a_joint3')])),
                            clock.sleep(dt))
    jog.run()
    assert jog.speed == len(lj.SPEEDS) - 1
    assert robot.stops == ['stopped (SPACE)'] and shown[-1] == 'stopped (SPACE)'
    after = [q for t, q in trace if t > 1.2]
    assert np.ptp(after) == 0.0  # no further command after the stop


def test_robot_not_following_holds_and_ends_the_session():
    clock = Clock()
    robot = SimRobot(clock, lag=0.5)
    jog, _ = session(robot, clock, held('e', 0.0, 2.0) + [(3.0, 'x')])
    with pytest.raises(lj.JogInterrupted, match='away from the command'):
        jog.run()
    assert robot.stops == ['robot did not follow the live command']


def test_no_joint_states_ends_the_session():
    clock = Clock()
    robot = SimRobot(clock, stale_after=0.5)
    jog, _ = session(robot, clock, held('e', 0.0, 2.0))
    with pytest.raises(lj.JogInterrupted, match='no /joint_states'):
        jog.run()


def test_ctrl_c_holds_the_robot():
    clock = Clock()
    robot = SimRobot(clock)
    jog, _ = session(robot, clock, held('e', 0.0, 2.0))

    def interrupt(dt):
        clock.sleep(dt)
        if clock() > 0.5:
            raise KeyboardInterrupt
    jog.sleep = interrupt
    with pytest.raises(KeyboardInterrupt):
        jog.run()
    assert robot.stops == ['interrupted (Ctrl-C)']


def test_a_joint_outside_its_safe_band_is_refused():
    clock = Clock()
    q = np.zeros(14)
    q[index('arm_a_joint3')] = 1.0  # on the limit
    with pytest.raises(m.MotionError, match='return home first'):
        session(SimRobot(clock, q=q), clock, [])


def test_exit_while_moving_holds():
    clock = Clock()
    robot = SimRobot(clock)
    jog, _ = session(robot, clock, held('e', 0.0, 2.0) + [(1.0, '\x1b')])
    jog.run()
    assert robot.stops == ['left live control']


def test_key_map_and_screen():
    keys = lj.key_map(['joint1', 'joint2'])
    assert keys == {'q': ('joint1', 1), 'a': ('joint1', -1), 'w': ('joint2', 1),
                    's': ('joint2', -1)}
    text = lj.render('Left arm', [('Q', 'A', 'joint1', 'shoulder yaw', 4.9, -89, 89)], 0.2,
                     'ready')
    assert 'Q / A  joint1' in text and 'hold a key to move' in text and 'SPACE stop' in text


# --- two-point command ----------------------------------------------------------------------
def test_command_ends_at_rest_beyond_the_moving_point():
    lim = limits()
    v = np.zeros(14)
    v[2] = 0.3
    ahead, vel, rest, t_rest = lj.stream_points(np.zeros(14), v, lj.HORIZON, lim)
    assert ahead[2] == pytest.approx(0.3 * lj.HORIZON) and vel[2] == 0.3
    assert rest[2] > ahead[2] and np.delete(rest, 2).max() == 0.0
    assert t_rest > lj.HORIZON


def test_command_rest_is_the_collision_checked_pose():
    """The joint brakes onto exactly the pose LiveJogger checked (CHECK_AHEAD == HORIZON)."""
    lim = limits()
    r = lj.Ramp(0.1, -0.99, 0.99, accel=lj.ACCEL_FRACTION * 3.0, velocity=-0.4,
                lead=lj.HORIZON)
    v = np.zeros(14)
    v[5] = r.velocity
    ref = np.zeros(14)
    ref[5] = r.position
    _, _, rest, _ = lj.stream_points(ref, v, lj.HORIZON, lim)
    assert rest[5] == pytest.approx(r.braking_point(lj.CHECK_AHEAD))


def test_command_near_the_limit_is_clipped_and_stops_there():
    lim = limits()
    v = np.zeros(14)
    v[0] = 0.5
    ref = np.zeros(14)
    ref[0] = lim.safe_upper[0] - 0.01
    ahead, vel, rest, _ = lj.stream_points(ref, v, lj.HORIZON, lim)
    assert ahead[0] == rest[0] == lim.safe_upper[0] and vel[0] == 0.0


def test_ramp_with_lead_keeps_the_streamed_stop_inside_the_band():
    """Braking early enough that reference + lead + braking never needs clipping."""
    lim = limits()
    r = lj.Ramp(0.0, float(lim.safe_lower[0]), float(lim.safe_upper[0]),
                accel=lj.ACCEL_FRACTION * 3.0, lead=lj.HORIZON)
    worst = 0.0
    for _ in range(300):
        r.step(0.03, 0.5)  # a tick: 0.05 s wall at RTF 0.6
        unclipped = r.position + r.velocity * lj.HORIZON + \
            r.velocity * abs(r.velocity) / (2 * r.accel)
        worst = max(worst, unclipped - r.upper)
    assert worst < 2e-3 and r.position == pytest.approx(r.upper, abs=5e-3)


def test_reaching_the_limit_after_letting_go_still_says_so():
    """The key is released while braking into the edge: the message must still appear."""
    clock = Clock()
    q = np.zeros(14)
    q[index('arm_a_joint3')] = 0.98
    robot = SimRobot(clock, q=q)
    jog, shown = session(robot, clock, [(0.0, 'e'), (3.0, 'x')])  # one tap: let go at once
    jog.run()
    assert robot.q[index('arm_a_joint3')] == pytest.approx(robot.limits.safe_upper[0])
    assert shown[-1] == 'arm_a_joint3 at the end of the safe range'
