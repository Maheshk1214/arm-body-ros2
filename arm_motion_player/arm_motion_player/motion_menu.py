"""
Motion library in the menu: listing, parameter text and the selected arm.

Pure Python. The motions themselves are arm_motion_demos' YAML files
(motion_library); this module turns what the user types into the values
those motions take, e.g. "repetitions=4 tempo=40" for one motion.
"""
import re
from typing import Any, Dict, Sequence

from arm_motion_demos import motion_library as ml
from arm_motion_demos import motions as m


def parse_values(text: str, spec: ml.MotionSpec) -> Dict[str, Any]:
    """
    'repetitions=4 tempo=40' (spaces or commas) -> {'repetitions': 4, 'tempo': 40.0}.

    Names and values are checked against the motion's parameters.
    """
    values: Dict[str, Any] = {}
    for part in re.split(r'[\s,]+', text.strip()):
        if not part:
            continue
        if '=' not in part:
            raise m.MotionError(f'"{part}": write parameters as name=value, e.g. '
                                f'{_example(spec)}')
        name, raw = (s.strip() for s in part.split('=', 1))
        if name not in spec.parameters:
            known = ', '.join(spec.parameters) or 'none'
            raise m.MotionError(f'{spec.name} has no parameter "{name}" (parameters: {known})')
        values[name] = _convert(raw, spec.parameters[name].kind)
    spec.resolve(values)  # range and type checks, with the motion's own messages
    return values


def _convert(raw: str, kind: str):
    if kind == 'choice':
        return raw
    try:
        number = float(raw)
    except ValueError:
        return raw  # resolve() refuses it with the parameter's own message
    return int(number) if kind == 'int' and number.is_integer() else number


def _example(spec: ml.MotionSpec) -> str:
    return ' '.join(f'{p.name}={p.default}' for p in spec.parameters.values()) or '(none)'


def arm_values(spec: ml.MotionSpec, arm_prefixes: Sequence[str]) -> Dict[str, Any]:
    """
    Return the selected arm for motions with an arm choice (e.g. wave's arm).

    A choice parameter whose options include the selected arm prefix (one
    arm selected in the menu) gets that arm; with both arms selected, the
    motion's own default is kept.
    """
    if len(arm_prefixes) != 1:
        return {}
    return {p.name: arm_prefixes[0] for p in spec.parameters.values()
            if p.kind == 'choice' and arm_prefixes[0] in p.choices}


def render_library(names: Sequence[str], library: Dict[str, ml.MotionSpec],
                   problems: Sequence[str]) -> str:
    rows = [' Motion library:']
    for k, name in enumerate(names, 1):
        spec = library[name]
        rows.append(f'  [{k}] {spec.title:<18} {spec.description}')
        if spec.parameters:
            rows.append('      ' + '  '.join(p.describe() for p in spec.parameters.values()))
    for problem in problems:
        rows.append(f'  (skipped) {problem}')
    rows.append('  [B] Back')
    return '\n'.join(rows)
