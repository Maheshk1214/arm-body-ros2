"""
YAML motion library: gestures described as data, built into validated motions.

Pure Python (numpy, yaml; no ROS). Every *.yaml file in the library folders
is one motion: its tunable parameters (with types and ranges), named poses
and a list of steps. The built-in motions live in this package's motions/
folder; motion_demo and the arm motion player both read them, so the
YAML files are the only definition of wave, dual_arm and dance67.

File format (see motions/*.yaml for complete examples):

    name: wave                       # unique; defaults to the file name
    title: Wave
    description: One arm out to the side ...
    parameters:                      # optional; each needs a default
      repetitions: {type: int, default: 3, min: 1, max: 20, help: ...}
      tempo: {type: float, default: 54.0, min: 10, max: 200, unit: bpm}
      arm: {type: choice, default: arm_a, choices: [arm_a, arm_b]}
    poses:                           # home (the SRDF pose) plus these changes
      hand_up:
        "{arm}": [1.3, -1.57, -1.2, -0.3, 1.57, 0.01]   # joint1..joint6
        "{arm}_gripper_joint": 0.6
    steps:
      - move: hand_up                # a pose name, or home
        label: "{arm}: raise hand"   # optional, {parameter} replaced
        duration: 2.0                # optional, s at speed 1.0
      - oscillate:
          amplitude: {"{arm}": [0, 0, 0.25, 0.2, 0.25, 0]}
          phase: {arm_b: pi}         # optional, rad
          period: 2 * 60 / tempo     # optional, s; default: fastest within the limits
          cycles: repetitions
          max_acceleration: 0.85     # optional, share of the acceleration limits
          too_fast: lower the tempo  # optional hint when max_acceleration is exceeded
      - hold: 0.5                    # s at speed 1.0
      - repeat: repetitions          # {i} = 1, 2, ... in the labels inside
        steps: [...]

Joints are named in pose and amplitude/phase tables by full joint name
(arm_a_joint2), by arm (arm_a, arm_b, both_arms: six values for joint1..joint6,
or one value for all six) or grippers ([arm_a, arm_b], or one value). Keys
may contain {parameter}. Numbers may be expressions of the parameters with
+ - * / **, parentheses, pi, abs/min/max/sqrt.

A motion must start and end at home, and must pass validate() (limits,
velocities, accelerations) before it is used.
"""
import ast
from dataclasses import dataclass, field
import math
import operator
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from arm_motion_demos import motions as m
import numpy as np
import yaml

ARMS = ('arm_a', 'arm_b')
ARM_JOINTS = {'arm_a': m.ARM_A, 'arm_b': m.ARM_B, 'both_arms': None}
TOP_KEYS = {'name', 'title', 'description', 'parameters', 'poses', 'steps'}
PARAM_KEYS = {'type', 'default', 'min', 'max', 'choices', 'help', 'unit'}
OSC_KEYS = {'amplitude', 'phase', 'period', 'cycles', 'max_acceleration', 'too_fast'}
STEP_KINDS = ('move', 'oscillate', 'hold', 'repeat')
MAX_REPEAT = 100
HOME_MATCH = 1e-9  # rad: a motion must end this close to home


class MotionFileError(m.MotionError):
    """A motion file is missing a value, has an invalid one, or cannot be built."""


# --- expressions --------------------------------------------------------------------
_BINARY = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
           ast.Div: operator.truediv, ast.Pow: operator.pow}
_UNARY = {ast.USub: operator.neg, ast.UAdd: operator.pos}
_FUNCTIONS = {'abs': abs, 'min': min, 'max': max, 'sqrt': math.sqrt}
_CONSTANTS = {'pi': math.pi}


def evaluate(value, names: Dict[str, Any], where: str) -> float:
    """Return a number, or the value of a small expression of `names` (parameters, i)."""
    if isinstance(value, bool):
        raise MotionFileError(f'{where}: expected a number, got {value!r}')
    if isinstance(value, (int, float)):
        return float(value)
    if not isinstance(value, str):
        raise MotionFileError(f'{where}: expected a number or expression, got {value!r}')
    try:
        tree = ast.parse(value, mode='eval')
    except SyntaxError:
        raise MotionFileError(f'{where}: cannot read {value!r} as a number or expression')

    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)) \
                and not isinstance(node.value, bool):
            return float(node.value)
        if isinstance(node, ast.Name):
            if node.id in names:
                v = names[node.id]
                if isinstance(v, bool) or not isinstance(v, (int, float)):
                    raise MotionFileError(f'{where}: {node.id} is not a number')
                return float(v)
            if node.id in _CONSTANTS:
                return _CONSTANTS[node.id]
            raise MotionFileError(f'{where}: unknown name {node.id!r} in {value!r}')
        if isinstance(node, ast.BinOp) and type(node.op) in _BINARY:
            try:
                return _BINARY[type(node.op)](ev(node.left), ev(node.right))
            except (ZeroDivisionError, OverflowError) as e:
                raise MotionFileError(f'{where}: {value!r}: {e}')
        if isinstance(node, ast.UnaryOp) and type(node.op) in _UNARY:
            return _UNARY[type(node.op)](ev(node.operand))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
                and node.func.id in _FUNCTIONS and not node.keywords:
            try:
                return float(_FUNCTIONS[node.func.id](*[ev(a) for a in node.args]))
            except (TypeError, ValueError) as e:
                raise MotionFileError(f'{where}: {value!r}: {e}')
        raise MotionFileError(f'{where}: {value!r} is not a supported expression '
                              '(numbers, parameters, + - * / **, pi, abs/min/max/sqrt)')
    result = ev(tree)
    if not math.isfinite(result):
        raise MotionFileError(f'{where}: {value!r} is not a finite number')
    return result


def _format(text, names: Dict[str, Any], where: str) -> str:
    try:
        return str(text).format(**names)
    except (KeyError, IndexError, ValueError) as e:
        raise MotionFileError(f'{where}: cannot fill in {text!r} ({e})')


# --- parameters ---------------------------------------------------------------------
@dataclass(frozen=True)
class Parameter:
    name: str
    kind: str                  # 'int', 'float' or 'choice'
    default: Any
    low: Optional[float] = None
    high: Optional[float] = None
    choices: Tuple[str, ...] = ()
    hint: str = ''    # the file's help text
    unit: str = ''

    def check(self, value, motion: str):
        """Return `value` converted to this parameter's type, or raise MotionError."""
        label = f'{motion} {self.name}'
        if self.kind == 'int':
            if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                    not math.isfinite(value):
                raise m.MotionError(f'{label} must be a whole number in '
                                    f'[{self.low:g}, {self.high:g}], got {value!r}')
            return m.check_int(label, value, int(self.low), int(self.high))
        if self.kind == 'float':
            return m.check_float(label, value, self.low, self.high)
        if value not in self.choices:
            raise m.MotionError(f'{label} must be one of {", ".join(self.choices)}, '
                                f'got {value!r}')
        return value

    def describe(self) -> str:
        if self.kind == 'choice':
            options = '|'.join(self.choices)
        else:
            options = f'{self.low:g}..{self.high:g}'
        unit = f' {self.unit}' if self.unit else ''
        return f'{self.name}={self.default}{unit} ({options})'


def _parameter(name: str, entry, where: str) -> Parameter:
    if not isinstance(entry, dict):
        raise MotionFileError(f'{where}: must be a mapping with type and default')
    unknown = set(entry) - PARAM_KEYS
    if unknown:
        raise MotionFileError(f'{where}: unknown key(s) {", ".join(sorted(unknown))}')
    kind = entry.get('type')
    if kind not in ('int', 'float', 'choice'):
        raise MotionFileError(f'{where}.type must be int, float or choice, got {kind!r}')
    if 'default' not in entry:
        raise MotionFileError(f'{where}: missing "default"')
    if kind == 'choice':
        choices = entry.get('choices')
        if not isinstance(choices, list) or not choices or \
                not all(isinstance(c, str) for c in choices):
            raise MotionFileError(f'{where}.choices must be a list of names')
        p = Parameter(name, kind, entry['default'], choices=tuple(choices),
                      hint=str(entry.get('help', '')))
    else:
        for key in ('min', 'max'):
            v = entry.get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise MotionFileError(f'{where}.{key} must be a number')
        if entry['min'] > entry['max']:
            raise MotionFileError(f'{where}: min is above max')
        if kind == 'int' and not all(float(entry[k]).is_integer() for k in ('min', 'max')):
            raise MotionFileError(f'{where}: min and max of an int must be whole numbers')
        p = Parameter(name, kind, entry['default'], float(entry['min']), float(entry['max']),
                      hint=str(entry.get('help', '')), unit=str(entry.get('unit', '')))
    try:
        p.check(p.default, where)
    except m.MotionError as e:
        raise MotionFileError(f'{where}: default is invalid: {e}')
    return p


# --- joint tables -------------------------------------------------------------------
def _joint_values(table, names: Dict[str, Any], where: str) -> Dict[int, float]:
    """{joint index: value} from a pose / amplitude / phase table."""
    if not isinstance(table, dict):
        raise MotionFileError(f'{where}: must be a mapping of joints to values')
    out: Dict[int, float] = {}
    for raw_key, value in table.items():
        key = _format(raw_key, names, f'{where} key')
        here = f'{where}.{key}'
        if key in ARM_JOINTS:
            joints = m.ARM_A + m.ARM_B if key == 'both_arms' else ARM_JOINTS[key]
            values = value if isinstance(value, list) else [value] * 6
            if len(values) != 6:
                raise MotionFileError(f'{here}: needs 6 values (joint1..joint6), '
                                      f'got {len(values)}')
            values = [evaluate(v, names, here) for v in values] * (len(joints) // 6)
        elif key == 'grippers':
            joints = m.GRIPPERS
            values = value if isinstance(value, list) else [value] * 2
            if len(values) != 2:
                raise MotionFileError(f'{here}: needs 2 values (arm_a, arm_b)')
            values = [evaluate(v, names, here) for v in values]
        elif key in m.INDEX:
            joints, values = [key], [evaluate(value, names, here)]
        else:
            raise MotionFileError(f'{here}: unknown joint or group (use a joint name, '
                                  'arm_a, arm_b, both_arms or grippers)')
        for j, v in zip(joints, values):
            out[m.INDEX[j]] = v
    return out


def _vector(table, names, where, base=None) -> np.ndarray:
    q = np.zeros(len(m.JOINTS)) if base is None else np.array(base, float)
    for i, v in _joint_values(table, names, where).items():
        q[i] = v
    return q


# --- the motion ---------------------------------------------------------------------
@dataclass
class MotionSpec:
    name: str
    title: str
    description: str
    path: Optional[Path]
    parameters: Dict[str, Parameter]
    poses: Dict[str, Any]
    steps: List[Any]
    problems: List[str] = field(default_factory=list)

    def resolve(self, values: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Defaults overridden by `values`, each checked; unknown names are refused."""
        values = dict(values or {})
        unknown = set(values) - set(self.parameters)
        if unknown:
            known = ', '.join(self.parameters) or 'none'
            raise m.MotionError(f'{self.name} has no parameter {", ".join(sorted(unknown))} '
                                f'(parameters: {known})')
        return {n: p.check(values.get(n, p.default), self.name)
                for n, p in self.parameters.items()}

    def build(self, home, limits: m.Limits, values=None, speed: float = 1.0) -> m.Motion:
        """Build the motion from home; raises MotionError if it is not a valid motion."""
        speed = m.check_float('speed', speed, 0.1, 3.0)
        names = self.resolve(values)
        where = self.path.name if self.path else self.name
        home = np.asarray(home, float)
        poses = {'home': home}
        for pose_name, table in self.poses.items():
            poses[pose_name] = _vector(table, names, f'{where}: poses.{pose_name}', home)
        b = m.MotionBuilder(home, limits, speed)
        self._run(self.steps, b, poses, names, limits, speed, f'{where}: steps')
        motion = b.build(self.name)
        if np.abs(motion.q[-1] - home).max() > HOME_MATCH:
            raise MotionFileError(f'{where}: the motion must end at home (add "- move: home" '
                                  'as the last step)')
        return motion

    def _run(self, steps, b, poses, names, limits, speed, where):
        for k, step in enumerate(steps):
            here = f'{where}[{k}]'
            label = _format(step['label'], names, f'{here}.label') if 'label' in step else None
            if 'move' in step:
                target = step['move']
                duration = None
                if 'duration' in step:
                    duration = evaluate(step['duration'], names, f'{here}.duration') / speed
                    if duration <= 0:
                        raise MotionFileError(f'{here}.duration must be positive')
                b.move_to(poses[target], duration=duration, label=label)
            elif 'hold' in step:
                seconds = evaluate(step['hold'], names, f'{here}.hold')
                if seconds <= 0:
                    raise MotionFileError(f'{here}.hold must be positive')
                b.hold(seconds, label=label)
            elif 'oscillate' in step:
                self._oscillate(step['oscillate'], b, names, limits, label, f'{here}.oscillate')
            else:  # repeat
                times = evaluate(step['repeat'], names, f'{here}.repeat')
                if times != int(times) or not 1 <= times <= MAX_REPEAT:
                    raise MotionFileError(f'{here}.repeat must be a whole number in '
                                          f'1..{MAX_REPEAT}, got {times:g}')
                for i in range(1, int(times) + 1):
                    self._run(step['steps'], b, poses, dict(names, i=i), limits, speed,
                              f'{here}.steps')

    def _oscillate(self, osc, b, names, limits, label, where):
        amplitude = _vector(osc['amplitude'], names, f'{where}.amplitude')
        phase = _vector(osc['phase'], names, f'{where}.phase') if 'phase' in osc else None
        cycles = evaluate(osc['cycles'], names, f'{where}.cycles')
        if cycles != int(cycles) or cycles < 1:
            raise MotionFileError(f'{where}.cycles must be a whole number >= 1, '
                                  f'got {cycles:g}')
        period = None
        if 'period' in osc:
            period = evaluate(osc['period'], names, f'{where}.period')
            if period <= 0:
                raise MotionFileError(f'{where}.period must be positive')
        if 'max_acceleration' in osc:
            share = evaluate(osc['max_acceleration'], names, f'{where}.max_acceleration')
            if not 0 < share <= 1:
                raise MotionFileError(f'{where}.max_acceleration must be in (0, 1]')
            if period is not None:
                need = np.abs(amplitude) * (2 * math.pi / period) ** 2 / limits.acceleration
                worst = int(np.argmax(need))
                if need[worst] > share:
                    shortest = period * math.sqrt(need[worst] / share)
                    hint = f': {osc["too_fast"]}' if 'too_fast' in osc else ''
                    raise m.MotionError(
                        f'{self.name}: {label or "oscillation"} is too fast: '
                        f'{m.JOINTS[worst]} would need {need[worst]:.0%} of its acceleration '
                        f'limit, {share:.0%} allowed (period {period:.2f} s, at least '
                        f'{shortest:.2f} s){hint}')
        b.oscillate(amplitude, int(cycles), period=period, phase=phase, label=label)


# --- reading files --------------------------------------------------------------------
def _steps(raw, poses, params, where) -> List[dict]:
    if not isinstance(raw, list) or not raw:
        raise MotionFileError(f'{where}: must be a non-empty list of steps')
    out = []
    for k, step in enumerate(raw):
        here = f'{where}[{k}]'
        if not isinstance(step, dict):
            raise MotionFileError(f'{here}: must be a mapping like {{move: home}}')
        kinds = [s for s in STEP_KINDS if s in step]
        if len(kinds) != 1:
            raise MotionFileError(f'{here}: needs exactly one of {", ".join(STEP_KINDS)}')
        kind = kinds[0]
        allowed = {kind, 'label'} | ({'duration'} if kind == 'move' else set()) | \
            ({'steps'} if kind == 'repeat' else set())
        unknown = set(step) - allowed
        if unknown:
            raise MotionFileError(f'{here}: unknown key(s) {", ".join(sorted(unknown))}')
        if kind == 'move' and step['move'] not in poses and step['move'] != 'home':
            known = ', '.join(['home', *poses])
            raise MotionFileError(f'{here}.move: unknown pose {step["move"]!r} '
                                  f'(poses: {known})')
        if kind == 'oscillate':
            osc = step['oscillate']
            if not isinstance(osc, dict):
                raise MotionFileError(f'{here}.oscillate: must be a mapping')
            unknown = set(osc) - OSC_KEYS
            if unknown:
                raise MotionFileError(f'{here}.oscillate: unknown key(s) '
                                      f'{", ".join(sorted(unknown))}')
            for key in ('amplitude', 'cycles'):
                if key not in osc:
                    raise MotionFileError(f'{here}.oscillate: missing "{key}"')
        if kind == 'repeat':
            if 'steps' not in step:
                raise MotionFileError(f'{here}: repeat needs "steps"')
            step = dict(step, steps=_steps(step['steps'], poses, params, f'{here}.steps'))
        out.append(step)
    return out


def load_motion(path: Path) -> MotionSpec:
    """Read and check one motion file (the structure; build() checks the result)."""
    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text())
    except OSError as e:
        raise MotionFileError(f'{path}: cannot read ({e.strerror})')
    except yaml.YAMLError as e:
        raise MotionFileError(f'{path.name}: not valid YAML: {e}')
    where = path.name
    if not isinstance(data, dict):
        raise MotionFileError(f'{where}: must contain a mapping (name, parameters, steps)')
    unknown = set(data) - TOP_KEYS
    if unknown:
        raise MotionFileError(f'{where}: unknown key(s) {", ".join(sorted(unknown))}')
    name = data.get('name', path.stem)
    if not isinstance(name, str) or not name.replace('_', '').isalnum():
        raise MotionFileError(f'{where}: name must be letters, digits and _ only, got {name!r}')
    if name == 'home':
        raise MotionFileError(f'{where}: "home" is reserved')
    raw_params = data.get('parameters') or {}
    if not isinstance(raw_params, dict):
        raise MotionFileError(f'{where}: parameters must be a mapping')
    params = {}
    for pname, entry in raw_params.items():
        if pname in ('i', 'pi', 'speed') or not str(pname).isidentifier():
            raise MotionFileError(f'{where}: parameters.{pname}: name not allowed')
        params[pname] = _parameter(pname, entry, f'{where}: parameters.{pname}')
    poses = data.get('poses') or {}
    if not isinstance(poses, dict) or 'home' in poses:
        raise MotionFileError(f'{where}: poses must be a mapping, and "home" is built in')
    steps = _steps(data.get('steps'), poses, params, f'{where}: steps')
    return MotionSpec(name, str(data.get('title', name)), str(data.get('description', '')),
                      path, params, poses, steps)


def builtin_folder() -> Path:
    """Return this package's motions/ folder (installed share, or the source tree)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('arm_motion_demos')) / 'motions'
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1] / 'motions'


def load_library(folders: Sequence[Path], home=None, limits=None
                 ) -> Tuple[Dict[str, MotionSpec], List[str]]:
    """
    Every *.yaml in `folders` (in order), as ({name: MotionSpec}, problems).

    A file that cannot be read, repeats a name or (with home and limits)
    does not build into a valid motion with its defaults is left out and
    described in problems, so one broken file does not hide the others.
    """
    library: Dict[str, MotionSpec] = {}
    problems: List[str] = []
    for folder in folders:
        folder = Path(folder)
        if not folder.is_dir():
            continue
        for path in sorted(folder.glob('*.yaml')):
            try:
                spec = load_motion(path)
                if spec.name in library:
                    raise MotionFileError(f'{path}: name {spec.name!r} is already used by '
                                          f'{library[spec.name].path}')
                if home is not None and limits is not None:
                    found = m.validate(spec.build(home, limits), limits)
                    if found:
                        raise MotionFileError(f'{path.name}: breaks the limits with its '
                                              'defaults: ' + '; '.join(found))
            except m.MotionError as e:
                problems.append(str(e))
                continue
            library[spec.name] = spec
    return library, problems


def build_sequence(library: Dict[str, MotionSpec], names: Sequence[str], home, limits,
                   values: Optional[Dict[str, Dict[str, Any]]] = None, speed: float = 1.0,
                   common: Optional[Dict[str, Any]] = None) -> List[Tuple[str, m.Motion]]:
    """
    Build and validate motions in order, as [(name, Motion)].

    `values` gives parameters per motion name; `common` parameters apply to
    every motion that has them (e.g. repetitions). 'home' entries are skipped
    (every motion starts and ends at home). Everything is checked before
    anything runs.
    """
    if not names:
        raise m.MotionError('sequence is empty')
    values, common = values or {}, common or {}
    out = []
    for name in names:
        if name == 'home':
            continue
        if name not in library:
            raise m.MotionError(f'unknown motion {name!r}; choose from '
                                f'{", ".join(sorted(library))} or home')
        spec = library[name]
        params = {k: v for k, v in common.items() if k in spec.parameters}
        params.update(values.get(name, {}))
        motion = spec.build(home, limits, params, speed)
        problems = m.validate(motion, limits)
        if problems:
            raise m.MotionError(f'{name} breaks the limits:\n  ' + '\n  '.join(problems))
        out.append((name, motion))
    return out
