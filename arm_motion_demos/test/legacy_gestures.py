"""
Frozen copy of the Python gestures that the YAML motion library replaced.

Kept only as the reference for test_motion_library.py: the YAML files must
build exactly these trajectories. Do not use it elsewhere.
"""
import math

from arm_motion_demos.motions import (ARM_B, check_float, check_int, INDEX, JOINTS,
                                      MotionBuilder, MotionError, pose)
import numpy as np

DANCE_OSC_FRACTION = 0.85
WAVE_POSE = [1.3, -1.57, -1.2, -0.3, 1.57, 0.01]
WAVE_SWAY = [0.0, 0.0, 0.25, 0.2, 0.25, 0.0]
FORWARD = [0.0, -1.57, 0.0, 0.0, 1.57, 0.01]
UP = [0.0, -2.7, 0.0, 0.0, 1.57, 0.01]
T_POSE = [1.3, -1.57, 0.0, 0.0, 1.57, 0.01]
DANCE_READY = [0.0, -0.35, -1.22, 0.0, 1.57, 0.01]
GRIPPER_OPEN = 0.6


def wave(home, limits, repetitions=3, speed=1.0, arm='arm_a'):
    repetitions = check_int('repetitions', repetitions, 1, 20)
    if arm not in ('arm_a', 'arm_b'):
        raise MotionError(f"wave arm must be 'arm_a' or 'arm_b', got {arm!r}")
    sl = slice(0, 6) if arm == 'arm_a' else slice(6, 12)
    up = np.array(home, float)
    up[sl] = WAVE_POSE
    up[12 if arm == 'arm_a' else 13] = GRIPPER_OPEN
    amp = np.zeros(len(JOINTS))
    amp[sl] = WAVE_SWAY
    b = MotionBuilder(home, limits, speed)
    b.move_to(up, label=f'{arm}: raise hand')
    b.oscillate(amp, repetitions, label=f'{arm}: wave x{repetitions}')
    b.move_to(home, label='back to home')
    return b.build('wave')


def dual_arm(home, limits, repetitions=2, speed=1.0):
    repetitions = check_int('repetitions', repetitions, 1, 20)
    forward = pose(home, FORWARD, FORWARD)
    up = pose(home, UP, UP)
    tpose = pose(home, T_POSE, T_POSE)
    b = MotionBuilder(home, limits, speed)
    b.move_to(forward, label='both arms forward')
    for i in range(repetitions):
        b.move_to(up, label=f'simultaneous raise {i + 1}/{repetitions}')
        b.move_to(forward)
    alt = np.zeros(len(JOINTS))
    alt[[INDEX['arm_a_joint2'], INDEX['arm_b_joint2']]] = 0.5
    phase = np.zeros(len(JOINTS))
    phase[INDEX['arm_b_joint2']] = math.pi
    b.oscillate(alt, repetitions, phase=phase, label=f'alternating arms x{repetitions}')
    b.move_to(tpose, label='arms out to the sides')
    flap = np.zeros(len(JOINTS))
    flap[[INDEX['arm_a_joint2'], INDEX['arm_b_joint2']]] = 0.4
    b.oscillate(flap, repetitions, label=f'symmetric flaps x{repetitions}')
    b.move_to(home, label='back to home')
    return b.build('dual_arm')


def _dance_amplitudes(amplitude):
    amp = np.zeros(len(JOINTS))
    for prefix in ('arm_a', 'arm_b'):
        amp[INDEX[f'{prefix}_joint2']] = 0.4 * amplitude
        amp[INDEX[f'{prefix}_joint3']] = amplitude
        amp[INDEX[f'{prefix}_joint5']] = 1.2 * amplitude
    return amp


def dance67_max_tempo(amplitude, limits):
    amp = _dance_amplitudes(amplitude)
    moving = amp > 0
    omega = np.min(np.sqrt(DANCE_OSC_FRACTION * limits.acceleration[moving] / amp[moving]))
    return 2 * 60 * omega / (2 * math.pi)


def dance67(home, limits, repetitions=6, tempo=54.0, amplitude=0.25, speed=1.0):
    repetitions = check_int('repetitions', repetitions, 1, 50)
    amplitude = check_float('dance amplitude', amplitude, 0.05, 0.3)
    max_tempo = dance67_max_tempo(amplitude, limits)
    tempo = check_float('dance tempo', tempo, 10.0, 200.0)
    if tempo > max_tempo:
        raise MotionError(f'dance tempo {tempo:.0f} bpm is too fast for amplitude {amplitude} '
                          f'rad: max {max_tempo:.0f} bpm within the acceleration limits')
    ready = pose(home, DANCE_READY, DANCE_READY, [GRIPPER_OPEN, GRIPPER_OPEN])
    amp = _dance_amplitudes(amplitude)
    sign = np.ones(len(JOINTS))
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
