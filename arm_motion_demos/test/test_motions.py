"""Offline tests of the gestures: limits, home, geometry and parameter checks."""
import math
import os
import sys

from arm_motion_demos import motions as m
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))

share = pytest.importorskip('ament_index_python.packages').get_package_share_directory
xacro = pytest.importorskip('xacro')
yaml = pytest.importorskip('yaml')


@pytest.fixture(scope='module')
def urdf():
    path = share('arm_body_description') + '/urdf/arm_body.urdf.xacro'
    return xacro.process_file(path).toxml()


@pytest.fixture(scope='module')
def limits(urdf):
    with open(share('arm_body_moveit_config') + '/config/joint_limits.yaml') as f:
        return m.parse_limits(urdf, yaml.safe_load(f))


@pytest.fixture(scope='module')
def home():
    with open(share('arm_body_moveit_config') + '/config/arm_body.srdf') as f:
        return m.parse_home(f.read())


@pytest.fixture(scope='module')
def gestures(home, limits):
    return {
        'wave_a': m.build_gesture('wave', home, limits, {'wave_arm': 'arm_a'}),
        'wave_b': m.build_gesture('wave', home, limits, {'wave_arm': 'arm_b'}),
        'dual_arm': m.build_gesture('dual_arm', home, limits, {}),
        'dance67': m.build_gesture('dance67', home, limits, {}),
    }


def joint(motion, name):
    return motion.q[:, m.INDEX[name]]


def phase_samples(motion, label):
    """Select the samples of the phase whose label starts with `label`."""
    starts = [t for t, _ in motion.phases] + [motion.duration]
    for i, (t, text) in enumerate(motion.phases):
        if text.startswith(label):
            return (motion.t >= t) & (motion.t < starts[i + 1])
    raise AssertionError(f'no phase {label!r} in {[p for _, p in motion.phases]}')


# --- home ------------------------------------------------------------------------
def test_home_is_the_srdf_home(home):
    expected = dict.fromkeys(m.JOINTS, 0.0)
    for arm in ('arm_a', 'arm_b'):
        expected.update({f'{arm}_joint2': -0.01, f'{arm}_joint5': 0.01,
                         f'{arm}_joint6': 0.01, f'{arm}_gripper_joint': 0.01})
    assert home == pytest.approx([expected[j] for j in m.JOINTS])


def test_home_is_inside_the_safe_limits(home, limits):
    assert m.validate(m.Motion(np.zeros(1), home[None, :], []), limits) == []


def test_limits_come_from_urdf_and_moveit(limits):
    i = m.INDEX['arm_a_joint2']
    assert (limits.lower[i], limits.upper[i]) == pytest.approx((-3.1416, 0.0))
    assert limits.velocity[i] == pytest.approx(7.85)
    assert limits.acceleration == pytest.approx(np.full(len(m.JOINTS), 3.0))


# --- every gesture ---------------------------------------------------------------
@pytest.mark.parametrize('name', ['wave_a', 'wave_b', 'dual_arm', 'dance67'])
def test_gesture_respects_limits(gestures, limits, name):
    assert m.validate(gestures[name], limits) == []


@pytest.mark.parametrize('name', ['wave_a', 'wave_b', 'dual_arm', 'dance67'])
def test_gesture_starts_and_ends_at_rest_at_home(gestures, home, name):
    motion = gestures[name]
    assert motion.q[0] == pytest.approx(home, abs=1e-12)
    assert motion.q[-1] == pytest.approx(home, abs=1e-12)
    vel = motion.velocities()
    assert np.abs(vel[:3]).max() < 0.01 and np.abs(vel[-3:]).max() < 0.01


@pytest.mark.parametrize('name', ['wave_a', 'wave_b', 'dual_arm', 'dance67'])
def test_gesture_waypoints_are_monotonic_and_end_at_rest(gestures, name):
    t, q, v, a = gestures[name].waypoints()
    assert t[0] == 0.0 and np.all(np.diff(t) > 0) and np.diff(t).max() <= 0.05 + 1e-9
    assert np.abs(v[-1]).max() == 0.0 and np.abs(a[-1]).max() == 0.0
    assert q[-1] == pytest.approx(gestures[name].q[-1])


@pytest.mark.parametrize('name', ['wave_a', 'wave_b', 'dual_arm', 'dance67'])
def test_gesture_clears_floor_body_and_other_arm(gestures, urdf, name):
    """Collision-mesh FK every 0.3 s: home itself hangs ~2 cm above the floor."""
    geometry = pytest.importorskip('geometry')
    robot = geometry.Robot(urdf, share('arm_body_description'))
    motion = gestures[name]
    worst = np.full(4, np.inf)
    for i in range(0, len(motion.t), 30):
        worst = np.minimum(worst, robot.clearances(dict(zip(m.JOINTS, motion.q[i]))))
    floor, arms, a_body, b_body = worst
    assert floor >= 0.015, f'floor clearance {floor:.3f} m'
    assert arms >= 0.05, f'arm-arm distance {arms:.3f} m'
    assert min(a_body, b_body) >= 0.02, f'arm-body distance {a_body:.3f}/{b_body:.3f} m'


# --- wave ------------------------------------------------------------------------
@pytest.mark.parametrize('arm,other', [('a', 'b'), ('b', 'a')])
def test_wave_moves_one_arm_only(gestures, home, arm, other):
    motion = gestures[f'wave_{arm}']
    idle = [m.INDEX[f'arm_{other}_joint{i}'] for i in range(1, 7)]
    assert np.abs(motion.q[:, idle] - home[idle]).max() < 1e-12
    assert joint(motion, f'arm_{arm}_joint1').max() > 1.0  # arm swung out to the side


def test_wave_repetitions_and_speed(home, limits):
    base = m.wave(home, limits, repetitions=2)
    more = m.wave(home, limits, repetitions=4)
    faster = m.wave(home, limits, repetitions=2, speed=1.1)
    sway = phase_samples(more, 'arm_a: wave')
    j3 = joint(more, 'arm_a_joint3')[sway]
    peaks = np.sum((j3[1:-1] > j3[:-2]) & (j3[1:-1] >= j3[2:]))
    assert peaks == 4 + 1  # one extra cycle ramps the sway in and out
    assert more.duration > base.duration > faster.duration
    assert m.validate(faster, limits) == []


# --- dual arm --------------------------------------------------------------------
def correlation(motion, label):
    mask = phase_samples(motion, label)
    a, b = joint(motion, 'arm_a_joint2')[mask], joint(motion, 'arm_b_joint2')[mask]
    return np.corrcoef(a - a.mean(), b - b.mean())[0, 1]


def test_dual_arm_simultaneous_alternating_symmetric(gestures):
    motion = gestures['dual_arm']
    assert correlation(motion, 'simultaneous raise') > 0.99
    assert correlation(motion, 'alternating arms') < -0.99
    assert correlation(motion, 'symmetric flaps') > 0.99
    out = phase_samples(motion, 'symmetric flaps')
    # joint1 is mirrored in the URDF: equal values = both arms out to the sides
    assert joint(motion, 'arm_a_joint1')[out] == pytest.approx(joint(motion, 'arm_b_joint1')[out])
    assert joint(motion, 'arm_a_joint1')[out].min() > 1.0


# --- 67 dance --------------------------------------------------------------------
def test_dance_alternates_hands_at_the_tempo(home, limits):
    tempo, reps = 40.0, 4
    motion = m.dance67(home, limits, repetitions=reps, tempo=tempo)
    mask = phase_samples(motion, 'dance: 6-7')
    a, b = joint(motion, 'arm_a_joint3')[mask], joint(motion, 'arm_b_joint3')[mask]
    assert np.corrcoef(a - a.mean(), b - b.mean())[0, 1] < -0.99
    peaks = np.flatnonzero((a[1:-1] < a[:-2]) & (a[1:-1] <= a[2:]))  # arm_a "up" = minima
    assert len(peaks) == reps + 1
    period = np.diff(peaks)[1:-1].mean() * m.DT  # the ramp cycles shift their peaks
    assert period == pytest.approx(2 * 60.0 / tempo, rel=0.02)
    grippers = motion.q[mask][:, [m.INDEX[g] for g in m.GRIPPERS]]
    assert grippers == pytest.approx(m.GRIPPER_OPEN)  # palms open while dancing


def test_dance_amplitude_scales_hand_motion(home, limits):
    def swing(amplitude):
        motion = m.dance67(home, limits, repetitions=2, tempo=30, amplitude=amplitude)
        j3 = joint(motion, 'arm_a_joint3')[phase_samples(motion, 'dance: 6-7')]
        return j3.max() - j3.min()
    assert swing(0.3) == pytest.approx(2 * swing(0.15), rel=0.02)


def test_the_largest_dance_amplitude_stays_within_every_limit(home, limits):
    tempo = m.dance67_max_tempo(0.3, limits) - 0.01
    assert m.validate(m.dance67(home, limits, tempo=tempo, amplitude=0.3), limits) == []
    with pytest.raises(m.MotionError, match='amplitude'):
        m.dance67(home, limits, tempo=20, amplitude=0.35)


def test_dance_too_fast_is_refused_with_the_limit(home, limits):
    max_tempo = m.dance67_max_tempo(0.25, limits)
    assert m.validate(m.dance67(home, limits, tempo=max_tempo - 1), limits) == []
    with pytest.raises(m.MotionError, match=f'max {max_tempo:.0f} bpm'):
        m.dance67(home, limits, tempo=max_tempo + 5)


# --- sequences -------------------------------------------------------------------
def test_sequence_order_and_home_entries(home, limits):
    seq = m.build_sequence(['home', 'dance67', 'wave', 'home', 'dual_arm', 'home'],
                           home, limits, {'repetitions': 1})
    assert [n for n, _ in seq] == ['dance67', 'wave', 'dual_arm']
    for _, motion in seq:  # every gesture hands over at home
        assert motion.q[0] == pytest.approx(home) and motion.q[-1] == pytest.approx(home)


def test_sequence_speed_changes_durations(home, limits):
    slow = m.build_sequence(['dual_arm'], home, limits, {'speed': 0.5})[0][1]
    normal = m.build_sequence(['dual_arm'], home, limits, {'speed': 1.0})[0][1]
    assert slow.duration > 1.8 * normal.duration


def test_sequence_refuses_motion_over_the_limits(home, limits):
    with pytest.raises(m.MotionError, match='acceleration limit'):
        m.build_sequence(['wave'], home, limits, {'speed': 2.5})


# --- invalid parameters ----------------------------------------------------------
@pytest.mark.parametrize('names,params,match', [
    ([], {}, 'empty'),
    (['moonwalk'], {}, 'unknown gesture'),
    (['wave'], {'repetitions': -1}, 'repetitions'),
    (['wave'], {'repetitions': 2.5}, 'repetitions'),
    (['wave'], {'repetitions': 100}, 'repetitions'),
    (['dual_arm'], {'repetitions': True}, 'repetitions'),
    (['wave'], {'speed': 0.0}, 'speed'),
    (['wave'], {'speed': math.nan}, 'speed'),
    (['wave'], {'speed': 'fast'}, 'speed'),
    (['wave'], {'wave_arm': 'arm_c'}, 'wave arm'),
    (['dance67'], {'dance_amplitude': 0.9}, 'amplitude'),
    (['dance67'], {'dance_tempo': -10}, 'tempo'),
    (['dance67'], {'dance_tempo': 300}, 'tempo'),
])
def test_invalid_parameters_are_refused(home, limits, names, params, match):
    with pytest.raises(m.MotionError, match=match):
        m.build_sequence(names, home, limits, params)


# --- validator -------------------------------------------------------------------
def test_validate_catches_positions_on_a_limit(home, limits):
    on_limit = home.copy()
    on_limit[m.INDEX['arm_b_joint2']] = 0.0  # old all-zero home: exactly on the limit
    motion = m.MotionBuilder(home, limits).move_to(on_limit).build()
    problems = m.validate(motion, limits)
    assert len(problems) == 1 and 'arm_b_joint2 above the safe upper limit' in problems[0]


def test_validate_catches_too_fast_moves(home, limits):
    target = home.copy()
    target[m.INDEX['arm_a_joint2']] = -1.5
    motion = m.MotionBuilder(home, limits).move_to(target, duration=0.4).build()
    assert any('acceleration' in p for p in m.validate(motion, limits))


def test_homing_from_a_limit_needs_allow_start(home, limits):
    zeros = np.zeros(len(m.JOINTS))  # e.g. left there by a topic command
    motion = m.home_motion(zeros, home, limits)
    assert m.validate(motion, limits) != []
    assert m.validate(motion, limits, allow_start=True) == []


def test_homing_never_allows_beyond_the_urdf_limit(home, limits):
    beyond = home.copy()
    beyond[m.INDEX['arm_a_joint5']] = -0.05
    motion = m.home_motion(beyond, home, limits)
    assert any('arm_a_joint5' in p for p in m.validate(motion, limits, allow_start=True))


def test_parse_errors_are_reported(urdf):
    with pytest.raises(m.MotionError, match='acceleration'):
        m.parse_limits(urdf, {'joint_limits': {}})
    with pytest.raises(m.MotionError, match='home'):
        m.parse_home('<robot name="x"/>')
