"""
Motion primitives and gestures for the dual-arm robot.

Pure Python (numpy, no ROS) so gestures can be built, validated and tested
offline. A motion is a densely sampled joint trajectory (every DT seconds)
for all 14 joints; gestures start and end at the SRDF home pose.

Safety rules applied by validate():
- positions stay at least LIMIT_MARGIN inside every URDF limit (home sits
  exactly on that margin for the limits at 0, which jam in Gazebo when held);
- velocities stay within the URDF limits;
- accelerations stay within MoveIt's joint_limits.yaml.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

ARM_A = [f'arm_a_joint{i}' for i in range(1, 7)]
ARM_B = [f'arm_b_joint{i}' for i in range(1, 7)]
GRIPPERS = ['arm_a_gripper_joint', 'arm_b_gripper_joint']
JOINTS = ARM_A + ARM_B + GRIPPERS
INDEX = {j: i for i, j in enumerate(JOINTS)}

DT = 0.01             # s, sample period of a motion
LIMIT_MARGIN = 0.01   # rad, kept from every URDF position limit
# Fractions of the velocity / acceleration limits used at speed 1.0. Moves can
# be sped up ~1.4x before validate() refuses them; oscillations (whose ramps add
# ~12 % to the peak acceleration) only ~1.1x.
VEL_FRACTION = 0.5
ACC_FRACTION = 0.5
OSC_FRACTION = 0.7
# The dance runs closer to the limit: its validated peak (ramp included) is ~95 %.
DANCE_OSC_FRACTION = 0.85


class MotionError(ValueError):
    """Invalid gesture parameters or a motion that breaks the limits."""


class Limits:
    """Per-joint limits, as arrays in JOINTS order."""

    def __init__(self, lower, upper, velocity, acceleration):
        self.lower = np.asarray(lower, float)
        self.upper = np.asarray(upper, float)
        self.velocity = np.asarray(velocity, float)
        self.acceleration = np.asarray(acceleration, float)

    @property
    def safe_lower(self):
        return self.lower + LIMIT_MARGIN

    @property
    def safe_upper(self):
        return self.upper - LIMIT_MARGIN


def parse_limits(urdf_xml, joint_limits):
    """
    Combine URDF position/velocity limits with MoveIt accelerations.

    `joint_limits` is the parsed joint_limits.yaml (dict).
    """
    urdf = {}
    for joint in ET.fromstring(urdf_xml).findall('joint'):  # not ros2_control's
        limit = joint.find('limit')
        if joint.get('name') in INDEX and limit is not None:
            urdf[joint.get('name')] = limit
    yaml_limits = joint_limits.get('joint_limits', {})
    missing = [j for j in JOINTS if j not in urdf]
    if missing:
        raise MotionError(f'URDF has no limits for {missing}')
    lower, upper, vel, acc = [], [], [], []
    for j in JOINTS:
        lower.append(float(urdf[j].get('lower')))
        upper.append(float(urdf[j].get('upper')))
        vel.append(float(urdf[j].get('velocity')))
        entry = yaml_limits.get(j, {})
        if not entry.get('has_acceleration_limits'):
            raise MotionError(f'joint_limits.yaml has no acceleration limit for {j}')
        acc.append(float(entry['max_acceleration']))
    return Limits(lower, upper, vel, acc)


def parse_home(srdf_xml):
    """Home pose: SRDF 'home' of both_arms plus the grippers' 'closed'."""
    states = {}
    for state in ET.fromstring(srdf_xml).iter('group_state'):
        states[(state.get('group'), state.get('name'))] = {
            j.get('name'): float(j.get('value')) for j in state.iter('joint')}
    home = {}
    for key in (('both_arms', 'home'), ('gripper_a', 'closed'),
                ('gripper_b', 'closed')):
        if key not in states:
            raise MotionError(f'SRDF has no group_state {key[1]!r} for {key[0]}')
        home.update(states[key])
    missing = [j for j in JOINTS if j not in home]
    if missing:
        raise MotionError(f'SRDF home/closed lacks {missing}')
    return np.array([home[j] for j in JOINTS])


def pose(home, arm_a=None, arm_b=None, grippers=None):
    """Home with the given arm (6 values) and gripper (2 values) overrides."""
    q = np.array(home, float)
    if arm_a is not None:
        q[0:6] = arm_a
    if arm_b is not None:
        q[6:12] = arm_b
    if grippers is not None:
        q[12:14] = grippers
    return q


class Motion:
    """Sampled trajectory: t (N,), q (N, 14), plus labelled phases for logging."""

    def __init__(self, t, q, phases, name=''):
        self.t, self.q, self.phases, self.name = t, q, phases, name

    @property
    def duration(self):
        return float(self.t[-1])

    def velocities(self):
        if len(self.q) < 2:  # a single pose
            return np.zeros_like(self.q)
        return np.gradient(self.q, DT, axis=0)

    def accelerations(self):
        if len(self.q) < 2:
            return np.zeros_like(self.q)
        return np.gradient(self.velocities(), DT, axis=0)

    def waypoints(self, step=0.05):
        """Every `step` s: (times, positions, velocities, accelerations)."""
        k = max(1, int(round(step / DT)))
        idx = np.arange(0, len(self.t), k)
        if idx[-1] != len(self.t) - 1:
            idx = np.append(idx, len(self.t) - 1)
        vel, acc = self.velocities(), self.accelerations()
        vel[-1] = acc[-1] = 0.0  # every motion ends at rest
        return self.t[idx], self.q[idx], vel[idx], acc[idx]

    def sample(self, t):
        """Planned positions at time t (clamped to the motion)."""
        i = min(max(int(round(t / DT)), 0), len(self.t) - 1)
        return self.q[i]


class MotionBuilder:
    """Appends smooth segments to a trajectory that starts at `start`."""

    def __init__(self, start, limits, speed=1.0):
        if not 0.0 < speed <= 3.0:
            raise MotionError(f'speed must be in (0, 3], got {speed}')
        self.limits, self.speed = limits, speed
        self.segments = [np.asarray(start, float)[None, :]]
        self.phases = []

    @property
    def current(self):
        return self.segments[-1][-1]

    @property
    def time(self):
        return (sum(len(s) for s in self.segments) - 1) * DT

    def _append(self, q, label):
        if label:
            self.phases.append((self.time, label))
        self.segments.append(q[1:])

    def move_to(self, target, duration=None, label=None):
        """Minimum-jerk move (zero velocity and acceleration at both ends)."""
        target = np.asarray(target, float)
        delta = np.abs(target - self.current)
        if duration is None:
            # min-jerk peaks: velocity 1.875*d/T, acceleration 5.774*d/T^2
            t_vel = 1.875 * delta / (VEL_FRACTION * self.limits.velocity)
            t_acc = np.sqrt(5.774 * delta / (ACC_FRACTION * self.limits.acceleration))
            duration = max(0.5, float(np.max(np.maximum(t_vel, t_acc)))) / self.speed
        n = max(2, int(round(duration / DT)) + 1)
        tau = np.linspace(0.0, 1.0, n)[:, None]
        s = tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)
        self._append(self.current + s * (target - self.current), label)
        return self

    def hold(self, duration, label=None):
        n = max(2, int(round(duration / self.speed / DT)) + 1)
        self._append(np.repeat(self.current[None, :], n, axis=0), label)
        return self

    def oscillate(self, amplitude, cycles, period=None, phase=None, label=None):
        """
        Oscillate: q = current + env(t) * amplitude * sin(2*pi*t/period + phase).

        The envelope ramps in over the first cycle and out over the last
        (C2-smooth), so the oscillation starts and ends at rest at the current
        pose. Without a period, the fastest one within OSC_FRACTION of the
        acceleration limits (scaled by speed) is used.
        """
        amplitude = np.asarray(amplitude, float)
        phase = np.zeros(len(JOINTS)) if phase is None else np.asarray(phase, float)
        if cycles < 1:
            raise MotionError(f'cycles must be >= 1, got {cycles}')
        if period is None:
            moving = amplitude > 0
            omega = np.min(np.sqrt(OSC_FRACTION * self.limits.acceleration[moving]
                                   / amplitude[moving]))
            period = 2 * math.pi / omega / self.speed
        cycles += 1  # one extra cycle is spent ramping in and out
        n = int(round(cycles * period / DT)) + 1
        t = np.linspace(0.0, cycles * period, n)[:, None]
        ramp = np.clip(np.minimum(t, t[-1] - t) / period, 0.0, 1.0)
        env = ramp ** 3 * (10 - 15 * ramp + 6 * ramp ** 2)
        wave = np.sin(2 * math.pi * t / period + phase) - np.sin(phase)
        self._append(self.current + env * amplitude * wave, label)
        return self

    def build(self, name=''):
        q = np.vstack(self.segments)
        return Motion(np.arange(len(q)) * DT, q, list(self.phases), name)


def validate(motion, limits, allow_start=False):
    """
    Problems found in `motion` (empty list = safe to command).

    allow_start: also accept positions between the start pose and the safe
    band (used to bring a robot that rests on a limit back to home).
    """
    problems = []
    lower, upper = limits.safe_lower.copy(), limits.safe_upper.copy()
    if allow_start:
        lower = np.maximum(np.minimum(lower, motion.q[0]), limits.lower)
        upper = np.minimum(np.maximum(upper, motion.q[0]), limits.upper)
    checks = (
        ('below the safe lower limit', motion.q, lambda x: x < lower - 1e-9, lower),
        ('above the safe upper limit', motion.q, lambda x: x > upper + 1e-9, upper),
        ('over the velocity limit', np.abs(motion.velocities()),
         lambda x: x > limits.velocity + 1e-6, limits.velocity),
        ('over the acceleration limit', np.abs(motion.accelerations()),
         lambda x: x > limits.acceleration + 1e-6, limits.acceleration),
    )
    for text, values, bad, bound in checks:
        mask = bad(values)
        for j in np.flatnonzero(mask.any(axis=0)):
            i = int(np.argmax(np.abs(values[:, j] - bound[j]) * mask[:, j]))
            problems.append(f'{JOINTS[j]} {text}: {values[i, j]:+.4f} '
                            f'(limit {bound[j]:+.4f}) at t={motion.t[i]:.2f} s')
    return problems


def check_int(name, value, low, high):
    if isinstance(value, bool) or int(value) != value or not low <= value <= high:
        raise MotionError(f'{name} must be an integer in [{low}, {high}], got {value}')
    return int(value)


def check_float(name, value, low, high):
    if not (isinstance(value, (int, float)) and math.isfinite(value)
            and low <= value <= high):
        raise MotionError(f'{name} must be a number in [{low}, {high}], got {value}')
    return float(value)


# Gesture poses (joint1..joint6). Robot faces +x; arm_a is its left arm.
# joint1 > 0 swings an arm outward; joint2 -pi/2 = arm forward, horizontal;
# joint5 = pi/2 lines the gripper up with the last arm segment. arm_b's joint1
# and joint6 axes are flipped in the URDF, so the same values on arm_b give the
# mirror-image pose.
WAVE_POSE = [1.3, -1.57, -1.2, -0.3, 1.57, 0.01]        # arm out to the side, forearm up
WAVE_SWAY = [0.0, 0.0, 0.25, 0.2, 0.25, 0.0]            # hand sways ~20 cm side to side
FORWARD = [0.0, -1.57, 0.0, 0.0, 1.57, 0.01]            # arm straight forward
UP = [0.0, -2.7, 0.0, 0.0, 1.57, 0.01]                  # arm raised high
T_POSE = [1.3, -1.57, 0.0, 0.0, 1.57, 0.01]             # arm straight out to the side
DANCE_READY = [0.0, -0.35, -1.22, 0.0, 1.57, 0.01]      # forearm forward, palm level
GRIPPER_OPEN = 0.6


def _arms(home, arm_a, arm_b, grippers=None):
    return pose(home, arm_a, arm_b, grippers)


def wave(home, limits, repetitions=3, speed=1.0, arm='arm_a'):
    """One arm out to the side with the forearm up, hand swaying side to side."""
    repetitions = check_int('repetitions', repetitions, 1, 20)
    if arm not in ('arm_a', 'arm_b'):
        raise MotionError(f"wave arm must be 'arm_a' or 'arm_b', got {arm!r}")
    sl = slice(0, 6) if arm == 'arm_a' else slice(6, 12)
    up = np.array(home, float)
    up[sl] = WAVE_POSE if arm == 'arm_a' else WAVE_POSE
    gr = 12 if arm == 'arm_a' else 13
    up[gr] = GRIPPER_OPEN
    amp = np.zeros(len(JOINTS))
    amp[sl] = WAVE_SWAY
    b = MotionBuilder(home, limits, speed)
    b.move_to(up, label=f'{arm}: raise hand')
    b.oscillate(amp, repetitions, label=f'{arm}: wave x{repetitions}')
    b.move_to(home, label='back to home')
    return b.build('wave')


def dual_arm(home, limits, repetitions=2, speed=1.0):
    """Both arms: raise together, alternate, then symmetric side flaps."""
    repetitions = check_int('repetitions', repetitions, 1, 20)
    forward = _arms(home, FORWARD, FORWARD)
    up = _arms(home, UP, UP)
    tpose = _arms(home, T_POSE, T_POSE)
    b = MotionBuilder(home, limits, speed)
    b.move_to(forward, label='both arms forward')
    for i in range(repetitions):
        b.move_to(up, label=f'simultaneous raise {i + 1}/{repetitions}')
        b.move_to(forward)
    alt = np.zeros(len(JOINTS))
    alt[[INDEX['arm_a_joint2'], INDEX['arm_b_joint2']]] = 0.5
    phase = np.zeros(len(JOINTS))
    phase[INDEX['arm_b_joint2']] = math.pi
    b.oscillate(alt, repetitions, phase=phase,
                label=f'alternating arms x{repetitions}')
    b.move_to(tpose, label='arms out to the sides')
    flap = np.zeros(len(JOINTS))
    flap[[INDEX['arm_a_joint2'], INDEX['arm_b_joint2']]] = 0.4
    b.oscillate(flap, repetitions, label=f'symmetric flaps x{repetitions}')
    b.move_to(home, label='back to home')
    return b.build('dual_arm')


def dance67_max_tempo(amplitude, limits):
    """Fastest tempo (beats/min) whose oscillation stays within DANCE_OSC_FRACTION."""
    amp = _dance_amplitudes(amplitude)
    moving = amp > 0
    omega = np.min(np.sqrt(DANCE_OSC_FRACTION * limits.acceleration[moving] / amp[moving]))
    return 2 * 60 * omega / (2 * math.pi)  # two beats ("6", "7") per cycle


def _dance_amplitudes(amplitude):
    """Per joint: forearm tilt = amplitude, small upper-arm bob, palm kept level."""
    amp = np.zeros(len(JOINTS))
    for prefix in ('arm_a', 'arm_b'):
        amp[INDEX[f'{prefix}_joint2']] = 0.4 * amplitude
        amp[INDEX[f'{prefix}_joint3']] = amplitude
        amp[INDEX[f'{prefix}_joint5']] = 1.2 * amplitude
    return amp


def dance67(home, limits, repetitions=6, tempo=54.0, amplitude=0.25, speed=1.0):
    """
    "6-7": forearms forward, palms up, hands bobbing up and down in turn.

    One cycle is two beats ("6" = left hand up, "7" = right hand up). The
    upper arm bobs with the forearm and the wrist counter-rotates, so the
    palms stay roughly level like weighing scales.
    """
    repetitions = check_int('repetitions', repetitions, 1, 50)
    # Above 0.3 rad the forearm (joint3) reaches its safe lower limit at any tempo.
    amplitude = check_float('dance amplitude', amplitude, 0.05, 0.3)
    max_tempo = dance67_max_tempo(amplitude, limits)
    tempo = check_float('dance tempo', tempo, 10.0, 200.0)
    if tempo > max_tempo:
        raise MotionError(
            f'dance tempo {tempo:.0f} bpm is too fast for amplitude {amplitude} '
            f'rad: max {max_tempo:.0f} bpm within the acceleration limits')
    ready = _arms(home, DANCE_READY, DANCE_READY,
                  [GRIPPER_OPEN, GRIPPER_OPEN])
    amp = _dance_amplitudes(amplitude)
    sign = np.ones(len(JOINTS))
    # "up" = forearm tilts up (joint2/joint3 more negative), wrist compensates
    for prefix in ('arm_a', 'arm_b'):
        sign[INDEX[f'{prefix}_joint2']] = sign[INDEX[f'{prefix}_joint3']] = -1
    phase = np.zeros(len(JOINTS))
    for j in ARM_B:
        phase[INDEX[j]] = math.pi
    b = MotionBuilder(home, limits, speed)
    b.move_to(ready, label='dance: ready, palms up')
    b.oscillate(sign * amp, repetitions, period=2 * 60.0 / tempo, phase=phase,
                label=f'dance: 6-7 x{repetitions} at {tempo:.0f} bpm')
    b.move_to(home, label='back to home')
    return b.build('dance67')


def home_motion(start, home, limits, speed=1.0):
    """Bring the robot from `start` to home."""
    return MotionBuilder(start, limits, speed).move_to(home, label='home').build('home')


GESTURES = ('wave', 'dual_arm', 'dance67')


def build_gesture(name, home, limits, params):
    """Build one gesture; params: speed, repetitions, wave_arm, dance_tempo, dance_amplitude."""
    speed = check_float('speed', params.get('speed', 1.0), 0.1, 3.0)
    reps = params.get('repetitions')
    if name == 'wave':
        return wave(home, limits, reps or 3, speed, params.get('wave_arm', 'arm_a'))
    if name == 'dual_arm':
        return dual_arm(home, limits, reps or 2, speed)
    if name == 'dance67':
        return dance67(home, limits, reps or 6, params.get('dance_tempo', 54.0),
                       params.get('dance_amplitude', 0.25), speed)
    raise MotionError(f'unknown gesture {name!r}; choose from {", ".join(GESTURES)} or home')


def build_sequence(names, home, limits, params):
    """
    Build and validate a sequence of gestures, as [(name, Motion)].

    'home' entries are skipped (every gesture starts and ends at home). Every
    motion is validated before anything runs.
    """
    if not names:
        raise MotionError('sequence is empty')
    motions = []
    for name in names:
        if name == 'home':
            continue
        motion = build_gesture(name, home, limits, params)
        problems = validate(motion, limits)
        if problems:
            raise MotionError(f'{name} breaks the limits:\n  ' + '\n  '.join(problems))
        motions.append((name, motion))
    return motions
