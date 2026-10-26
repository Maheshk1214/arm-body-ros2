"""
Single-joint (DOF) demonstrations, built from the robot's measured pose.

Pure Python. A DOF demo moves the selected joint (on one arm, or the same
joint on both arms) towards each of its limits and back, keeping every other
joint where it is. Each joint has its own range: the swing is clipped to
LIMIT_MARGIN inside that joint's URDF limits, and a side with too little room
is left out. The two sides are built separately so the caller can collision-
check each one and keep only the safe ones (combine()).

Jog moves are the user's own steps: jog_target() moves the selected joint(s)
by a step or to an angle, clipped to the same safe band.
"""
from dataclasses import dataclass
import math
from typing import List, Optional, Sequence

from arm_motion_demos import motions as m
import numpy as np

MIN_SWING = 0.05  # rad: a side with less room than this is skipped


@dataclass
class Side:
    label: str          # e.g. 'arm_a_joint3 to +0.50 rad'
    target: np.ndarray  # full 14-joint pose at the far end of the swing
    swing: float        # rad actually moved (after clipping)


def plan_sides(start: Sequence[float], joints: Sequence[str], amplitude: float,
               limits: m.Limits) -> List[Side]:
    """
    Up to two swings (towards the upper and the lower limit) of `joints`.

    Raises MotionError if a selected joint is outside its safe band (e.g. left
    on a limit by another tool): bring the robot home first.
    """
    start = np.asarray(start, float)
    if not 0.0 < amplitude <= 3.2:
        raise m.MotionError(f'amplitude must be in (0, 3.2] rad, got {amplitude}')
    idx = []
    for j in joints:
        if j not in m.INDEX:
            raise m.MotionError(f'unknown joint {j!r}')
        i = m.INDEX[j]
        if not limits.safe_lower[i] - 1e-6 <= start[i] <= limits.safe_upper[i] + 1e-6:
            raise m.MotionError(
                f'{j} is at {start[i]:+.3f} rad, within {m.LIMIT_MARGIN} rad of its limit '
                f'[{limits.lower[i]:+.3f}, {limits.upper[i]:+.3f}]: return home first')
        idx.append(i)
    sides = []
    for sign, word in ((+1, 'up'), (-1, 'down')):
        target = start.copy()
        swings = []
        for i in idx:
            goal = np.clip(start[i] + sign * amplitude, limits.safe_lower[i],
                           limits.safe_upper[i])
            target[i] = goal
            swings.append(abs(goal - start[i]))
        if min(swings) >= MIN_SWING:
            ends = ', '.join(f'{m.JOINTS[i]} {target[i]:+.2f}' for i in idx)
            sides.append(Side(f'{word}: {ends} rad', target, float(min(swings))))
    if not sides:
        raise m.MotionError(f'{", ".join(joints)}: no room to move within the safe range')
    return sides


def side_motion(start, side: Side, limits: m.Limits, speed: float) -> m.Motion:
    """Start -> far end of one side -> start (minimum-jerk moves)."""
    b = m.MotionBuilder(start, limits, speed)
    b.move_to(side.target, label=side.label).move_to(start, label='back')
    return b.build('dof')


def combine(start, sides: Sequence[Side], limits: m.Limits, speed: float,
            name: str) -> m.Motion:
    """One motion running the given sides in order, back to `start` after each."""
    b = m.MotionBuilder(start, limits, speed)
    for side in sides:
        b.move_to(side.target, label=side.label).move_to(start, label='back')
    return b.build(name)


# --- jog: the user steps one joint by hand -----------------------------------------
JOG_MIN_DURATION = 0.3  # s at speed 1.0: a small step should not take the usual 0.5 s
JOG_MIN_STEP = 1e-3     # rad: a request that moves less than this is refused


def jog_target(start: Sequence[float], joints: Sequence[str], limits: m.Limits,
               delta: Optional[float] = None, goal: Optional[float] = None):
    """
    Pose with `joints` moved by `delta` rad, or to the angle `goal` rad; others unchanged.

    Each joint is clipped to its own safe band. Returns (target, notes); notes say
    where a joint was stopped at its limit. Raises MotionError if nothing would move.
    """
    if (delta is None) == (goal is None):
        raise m.MotionError('give either a step or a target angle')
    start = np.asarray(start, float)
    target, notes, moved = start.copy(), [], False
    for j in joints:
        if j not in m.INDEX:
            raise m.MotionError(f'unknown joint {j!r}')
        i = m.INDEX[j]
        lo, hi = limits.safe_lower[i], limits.safe_upper[i]
        if not lo - 1e-6 <= start[i] <= hi + 1e-6:
            raise m.MotionError(f'{j} is at {start[i]:+.3f} rad, outside its safe range: '
                                'return home first')
        wanted = goal if goal is not None else start[i] + delta
        target[i] = float(np.clip(wanted, lo, hi))
        if abs(target[i] - wanted) > 1e-9:
            notes.append(f'{j} stopped at {math.degrees(target[i]):+.1f} deg, '
                         f'{m.LIMIT_MARGIN} rad inside its limit')
        moved = moved or abs(target[i] - start[i]) >= JOG_MIN_STEP
    if not moved:
        if goal is None:
            word = 'upper' if delta > 0 else 'lower'
            raise m.MotionError(f'{", ".join(joints)}: already at the {word} end of the '
                                'safe range')
        raise m.MotionError(f'{", ".join(joints)}: already there')
    return target, notes


def jog_motion(start, target, limits: m.Limits, speed: float) -> m.Motion:
    """One minimum-jerk move, as short as the velocity/acceleration limits allow."""
    start, target = np.asarray(start, float), np.asarray(target, float)
    delta = np.abs(target - start)
    t_vel = 1.875 * delta / (m.VEL_FRACTION * limits.velocity)
    t_acc = np.sqrt(5.774 * delta / (m.ACC_FRACTION * limits.acceleration))
    natural = max(JOG_MIN_DURATION, float(np.max(np.maximum(t_vel, t_acc))))
    return m.MotionBuilder(start, limits, speed).move_to(
        target, duration=natural / speed, label='jog').build('jog')
