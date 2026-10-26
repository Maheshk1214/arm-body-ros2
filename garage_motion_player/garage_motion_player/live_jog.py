"""
Live joint control: hold a key and the joint moves, let go and it stops.

A terminal sends no key-release events, only the key's auto-repeat (on this
desktop: after 500 ms, then 33 per second). KeyHold therefore treats a key as
held while its repeats keep coming: up to `first_gap` after the first press,
`repeat_gap` once it repeats.

The joint follows a reference that LiveJogger moves with a velocity ramp
(Ramp): it accelerates to the jog speed while a key is held, brakes in time
for the safe band (0.01 rad inside the URDF limits), and decelerates to rest
on release; SPACE stops at once. Each tick the reference plus a short lookahead is streamed to the
trajectory controller, and the pose the joint would reach if it braked now
(plus a margin) is collision-checked with MoveIt first: a direction that
would collide is stopped and blocked until the other direction is used. The
measured joints are compared with the reference; too far off, or no
/joint_states, and the robot holds.

Pure Python: the robot, the keys and the clock are passed in, so the loop can
be tested without ROS or a terminal.
"""
from dataclasses import dataclass
import math
import time
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from arm_motion_demos import motions as m
import numpy as np

# key pairs (up, down) for the joints in menu order: one row of the keyboard each
KEY_PAIRS = [('q', 'a'), ('w', 's'), ('e', 'd'), ('r', 'f'), ('t', 'g'), ('y', 'h'),
             ('u', 'j')]
STOP_KEYS = (' ',)
EXIT_KEYS = ('x', '\x1b')         # x or Esc
SLOWER_KEYS, FASTER_KEYS = ('[', '-'), (']', '+', '=')
SPEEDS = (0.05, 0.1, 0.2, 0.3, 0.5)  # rad/s, chosen with [ and ]
DEFAULT_SPEED = 2                   # index into SPEEDS
ACCEL_FRACTION = 0.5    # of each joint's acceleration limit, for the ramp
VEL_FRACTION = 0.5      # the jog speed never exceeds this share of the URDF velocity
HORIZON = 0.25          # s (sim): how far ahead each streamed point is
CHECK_AHEAD = 0.25      # s (sim) of motion beyond the braking distance, collision-checked
TICK = 0.05             # s (wall) between commands
STALE = 1.0             # s (wall) without /joint_states -> hold
EDGE_SNAP = 0.005       # rad: closer than this to the band edge, a moving joint ends on it


class KeyHold:
    """Which jog key is held, from a stream of key presses and their auto-repeats."""

    def __init__(self, first_gap: float = 0.6, repeat_gap: float = 0.15):
        self.first_gap, self.repeat_gap = first_gap, repeat_gap
        self.key: Optional[str] = None
        self.pressed = self.last = 0.0

    def press(self, key: str, now: float) -> None:
        if key != self.key:
            self.key, self.pressed = key, now
        self.last = now

    def release(self) -> None:
        self.key = None

    def held(self, now: float) -> Optional[str]:
        if self.key is None:
            return None
        repeating = self.last > self.pressed
        if now - self.last > (self.repeat_gap if repeating else self.first_gap):
            self.key = None
        return self.key


@dataclass
class Ramp:
    """Velocity-ramped reference for one joint, kept inside [lower, upper]."""

    position: float
    lower: float
    upper: float
    accel: float
    velocity: float = 0.0
    lead: float = 0.0  # s the streamed command runs ahead of the reference (HORIZON)

    def max_speed(self, room: float) -> float:
        """Fastest speed from which lead s of motion plus braking still fits in `room`."""
        # v * lead + v^2 / (2 a) = room
        a, h = self.accel, self.lead
        return a * (-h + math.sqrt(h * h + 2 * max(0.0, room) / a))

    def step(self, dt: float, wanted: float) -> None:
        """Advance by dt towards velocity `wanted`, braking in time for the band edges."""
        if dt <= 0.0:
            return
        # room left after this step, so a discrete step cannot overrun the curve
        travel = abs(self.velocity) * dt
        wanted = min(wanted, self.max_speed(self.upper - self.position - travel))
        wanted = max(wanted, -self.max_speed(self.position - self.lower - travel))
        change = np.clip(wanted - self.velocity, -self.accel * dt, self.accel * dt)
        self.velocity += float(change)
        self.position += self.velocity * dt
        # the speed limit near an edge falls towards zero, so the last few mrad would
        # creep: finish them in one step
        near_up = self.velocity > 0 and self.upper - self.position < EDGE_SNAP
        near_down = self.velocity < 0 and self.position - self.lower < EDGE_SNAP
        if near_up or near_down or not self.lower <= self.position <= self.upper:
            self.position = self.upper if self.velocity > 0 else self.lower
            self.velocity = 0.0

    def braking_point(self, extra: float) -> float:
        """Where the joint would stop if it braked now, plus `extra` s of motion."""
        v = self.velocity
        return self.position + math.copysign(v * v / (2 * self.accel) + abs(v) * extra, v)

    @property
    def moving(self) -> bool:
        return abs(self.velocity) > 1e-4


def stream_points(reference, velocity, horizon: float, limits: m.Limits):
    """
    Two-point command for the trajectory controllers (all 14 joints).

    Returns (ahead, ahead_velocity, rest, rest_time): where the reference will
    be in `horizon` s, then at rest where it brakes to from there. The
    controllers reject a trajectory that ends moving, and ending at rest means
    the joint brakes by itself if commands stop. `rest` is exactly the pose
    LiveJogger collision-checks when CHECK_AHEAD == HORIZON.
    """
    lo, hi = limits.safe_lower, limits.safe_upper
    velocity = np.asarray(velocity, float)
    decel = ACCEL_FRACTION * limits.acceleration
    wanted = np.asarray(reference, float) + velocity * horizon
    ahead = np.clip(wanted, lo, hi)
    brake_time = np.abs(velocity) / decel
    rest = np.clip(ahead + velocity * brake_time / 2, lo, hi)
    ahead_velocity = np.where(ahead != wanted, 0.0, velocity)
    return ahead, ahead_velocity, rest, horizon + max(0.05, float(brake_time.max()))


def key_map(shorts: Sequence[str]) -> Dict[str, Tuple[str, int]]:
    """Key -> (short joint name, direction +1/-1), for up to len(KEY_PAIRS) joints."""
    out = {}
    for short, (up, down) in zip(shorts, KEY_PAIRS):
        out[up], out[down] = (short, +1), (short, -1)
    return out


Row = Tuple[str, str, str, str, float, float, float]  # keys, joint, label, now, range (deg)


def render(arm_name: str, rows: Sequence[Row], speed: float, status: str) -> str:
    """Help screen: keys, joints with their angle (deg) and range."""
    lines = ['', '-' * 62, f' LIVE CONTROL  {arm_name}     hold a key to move, let go to stop',
             '   keys   joint      what            now (deg)    range (deg)']
    for up, down, short, label, now, lo, hi in rows:
        lines.append(f'   {up} / {down}  {short:<9}  {label:<15} {now:+8.1f}    '
                     f'{lo:+.0f} .. {hi:+.0f}')
    lines += [f' Speed {speed:.2f} rad/s   [ / ] slower / faster   SPACE stop   X (or Esc) '
              'back to the menu',
              f' {status}', '-' * 62]
    return '\n'.join(lines)


class JogInterrupted(Exception):
    """The live session ended because of a problem; the robot was told to hold."""


class LiveJogger:
    """
    One live-control session on the joints of `arms` (arm prefixes in config order).

    robot needs: limits, state() -> (stamp, q) or None, state_age() (wall s),
    stream(reference, velocity, horizon, joints, stamp), collision(q) -> str or None and
    stop(reason). keys() returns the characters typed since the last call;
    show(status) is called every tick and sleep(TICK) between ticks.
    """

    def __init__(self, robot, joint_names: Dict[str, List[str]], keys: Callable[[], str],
                 show: Callable[[str], None], tolerance: float = 0.2,
                 first_gap: float = 0.6, repeat_gap: float = 0.15,
                 clock: Callable[[], float] = time.monotonic, sleep=None):
        self.robot, self.joint_names = robot, joint_names  # short -> full names
        self.keys, self.show, self.tolerance = keys, show, tolerance
        self.clock = clock
        self.sleep = sleep
        self.hold = KeyHold(first_gap, repeat_gap)
        self.map = key_map(list(joint_names))
        self.speed = DEFAULT_SPEED
        self.blocked: Dict[str, int] = {}  # full joint -> direction refused (collision)
        self.status = 'ready'
        state = robot.state()
        if state is None:
            raise m.MotionError('no /joint_states received')
        lim = robot.limits
        self.start = np.clip(state[1], lim.safe_lower, lim.safe_upper)
        self.ramps = {}
        for names in joint_names.values():
            for j in names:
                i = m.INDEX[j]
                if not lim.safe_lower[i] - 0.003 <= state[1][i] <= lim.safe_upper[i] + 0.003:
                    raise m.MotionError(f'{j} is outside its safe range: return home first')
                self.ramps[j] = Ramp(float(self.start[i]), float(lim.safe_lower[i]),
                                     float(lim.safe_upper[i]),
                                     float(ACCEL_FRACTION * lim.acceleration[i]), lead=HORIZON)
        self.last_stamp = state[0]
        self.max_off = 0.0
        self.was_moving = False
        self.prev_moving: set = set()

    def jog_speed(self, joint: str) -> float:
        i = m.INDEX[joint]
        return min(SPEEDS[self.speed], VEL_FRACTION * float(self.robot.limits.velocity[i]))

    def reference(self) -> np.ndarray:
        q = self.start.copy()
        for j, r in self.ramps.items():
            q[m.INDEX[j]] = r.position
        return q

    def handle_keys(self, now: float) -> bool:
        """Read typed keys; False when the user leaves the session."""
        for k in self.keys():
            k = k.lower()
            if k in EXIT_KEYS:
                return False
            if k in STOP_KEYS:  # stop now (the controllers brake), not just "let go"
                self.hold.release()
                if any(r.moving for r in self.ramps.values()):
                    self.robot.stop('stopped (SPACE)')
                    self.sync(self.robot.state()[1])
                    self.was_moving = False
                self.status = 'stopped (SPACE)'
            elif k in SLOWER_KEYS:
                self.speed = max(0, self.speed - 1)
            elif k in FASTER_KEYS:
                self.speed = min(len(SPEEDS) - 1, self.speed + 1)
            elif k in self.map:
                short, sign = self.map[k]
                for j in self.joint_names[short]:
                    if self.blocked.get(j) == -sign:  # the other way frees a blocked side
                        del self.blocked[j]
                self.hold.press(k, now)
        return True

    def tick(self, now: float) -> None:
        """Advance the references, check them and stream them to the robot."""
        state = self.robot.state()
        if state is None or self.robot.state_age() > STALE:
            raise JogInterrupted('no /joint_states')
        dt, self.last_stamp = max(0.0, state[0] - self.last_stamp), state[0]
        key = self.hold.held(now)
        wanted = {}
        if key:
            short, sign = self.map[key]
            for j in self.joint_names[short]:
                wanted[j] = 0.0 if self.blocked.get(j) == sign else sign * self.jog_speed(j)
        for j, r in self.ramps.items():
            r.step(dt, wanted.get(j, 0.0))
        moving = [j for j, r in self.ramps.items() if r.moving]
        if moving:
            ahead = self.reference()
            for j in moving:
                ahead[m.INDEX[j]] = self.ramps[j].braking_point(CHECK_AHEAD)
            ahead = np.clip(ahead, self.robot.limits.safe_lower, self.robot.limits.safe_upper)
            problem = self.robot.collision(ahead)
            if problem:
                for j in moving:
                    self.blocked[j] = 1 if self.ramps[j].velocity > 0 else -1
                self.hold.release()
                self.robot.stop(f'collision ahead: {problem}')
                self.sync(self.robot.state()[1])
                self.was_moving = False
                self.status = f'stopped before a collision ({problem}); move the other way'
                return
        off = np.abs(state[1] - self.reference())
        self.max_off = max(self.max_off, float(off.max()))
        if off.max() > self.tolerance:
            self.robot.stop('robot did not follow the live command')
            raise JogInterrupted(f'{m.JOINTS[int(off.argmax())]} is {off.max():.2f} rad away '
                                 'from the command')
        if moving or self.was_moving:  # the last command after a move is "at rest here"
            velocity = np.zeros(len(m.JOINTS))
            for j in moving:
                velocity[m.INDEX[j]] = self.ramps[j].velocity
            self.robot.stream(self.reference(), velocity, HORIZON, list(self.ramps),
                              self.last_stamp)
        self.was_moving = bool(moving)
        if moving:
            self.prev_moving = set(moving)
            self.status = 'moving ' + ', '.join(
                f'{j} {"+" if self.ramps[j].velocity > 0 else "-"}' for j in moving)
            return
        # stopped on an edge: pushed against it now, or just arrived (key already let go)
        ends = [j for j, r in self.ramps.items() if not r.moving and
                (j in self.prev_moving or wanted.get(j)) and
                min(abs(r.position - r.upper), abs(r.position - r.lower)) < 1e-3]
        self.prev_moving = set()
        if ends:
            self.status = f'{", ".join(ends)} at the end of the safe range'
        elif self.status.startswith('moving'):
            self.status = 'stopped'

    def sync(self, q) -> None:
        """Restart every reference at rest from the measured pose (after a hold)."""
        q = np.clip(q, self.robot.limits.safe_lower, self.robot.limits.safe_upper)
        for j, r in self.ramps.items():
            r.position, r.velocity = float(q[m.INDEX[j]]), 0.0

    def run(self) -> float:
        """Run until the user exits; returns the largest command-to-robot difference."""
        try:
            while True:
                now = self.clock()
                if not self.handle_keys(now):
                    break
                self.tick(now)
                self.show(self.status)
                if self.sleep:
                    self.sleep(TICK)
        except KeyboardInterrupt:
            self.robot.stop('interrupted (Ctrl-C)')
            raise
        finally:
            self.hold.release()
        if any(r.moving for r in self.ramps.values()):
            self.robot.stop('left live control')
        return self.max_off
