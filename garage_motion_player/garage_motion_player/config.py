"""Load and check config/player.yaml (pure Python)."""
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import yaml


class ConfigError(ValueError):
    """config/player.yaml is missing a value or has an invalid one."""


@dataclass(frozen=True)
class JointInfo:
    short: str        # menu name, e.g. 'joint3'
    suffix: str       # full name = <arm prefix>_<suffix>
    label: str
    amplitude: float  # rad, single-joint demo swing


@dataclass(frozen=True)
class Config:
    ros_domain_id: int
    gz_partition: str
    gui: bool
    rviz: bool
    start_timeout: float
    speed: float
    tracking_tolerance: float
    first_gap: float              # s, live control: key released (no repeat yet)
    repeat_gap: float             # s, live control: key released (repeating)
    arms: Dict[str, str]          # 'left' -> 'arm_a'
    joints: List[JointInfo]
    dof_pose: Dict[str, float]    # short joint name -> rad, applied to both arms
    recordings_dir: Path
    logs_dir: Path

    def joint_name(self, arm: str, short: str) -> str:
        """Full joint name, e.g. ('left', 'joint3') -> 'arm_a_joint3'."""
        info = {j.short: j for j in self.joints}[short]
        return f'{self.arms[arm]}_{info.suffix}'


def _get(data, path, kind, check=None, hint=''):
    value = data
    for key in path.split('.'):
        if not isinstance(value, dict) or key not in value:
            raise ConfigError(f'player.yaml: missing "{path}"')
        value = value[key]
    if kind is float and isinstance(value, int) and not isinstance(value, bool):
        value = float(value)
    if not isinstance(value, kind) or (kind is not bool and isinstance(value, bool)):
        raise ConfigError(f'player.yaml: "{path}" must be a {kind.__name__}, got {value!r}')
    if check and not check(value):
        raise ConfigError(f'player.yaml: "{path}" = {value!r} is invalid{hint}')
    return value


def load_config(path: Path, workspace: Path) -> Config:
    """Read and validate the config; folders are resolved against `workspace`."""
    try:
        data = yaml.safe_load(Path(path).read_text())
    except FileNotFoundError:
        raise ConfigError(f'config file not found: {path}')
    except yaml.YAMLError as e:
        raise ConfigError(f'{path} is not valid YAML: {e}')
    if not isinstance(data, dict):
        raise ConfigError(f'{path} must contain a mapping of settings')
    arms = _get(data, 'arms', dict)
    if set(arms) != {'left', 'right'} or not all(isinstance(v, str) and v
                                                 for v in arms.values()):
        raise ConfigError('player.yaml: "arms" needs a prefix for exactly "left" and "right"')
    joints = []
    for short, entry in _get(data, 'joints', dict).items():
        if not isinstance(entry, dict):
            raise ConfigError(f'player.yaml: joints.{short} must be a mapping')
        joints.append(JointInfo(
            short=str(short),
            suffix=_get(data, f'joints.{short}.suffix', str),
            label=_get(data, f'joints.{short}.label', str),
            amplitude=_get(data, f'joints.{short}.amplitude', float, lambda v: 0.0 < v <= 3.2,
                           ' (0 < amplitude <= 3.2 rad)')))
    if not joints:
        raise ConfigError('player.yaml: "joints" is empty')
    dof_pose = _get(data, 'dof_pose', dict)
    shorts = {j.short for j in joints}
    if set(dof_pose) != shorts:
        raise ConfigError(f'player.yaml: "dof_pose" needs exactly the joints {sorted(shorts)}')
    for short in dof_pose:
        _get(data, f'dof_pose.{short}', float)
    return Config(
        ros_domain_id=_get(data, 'ros_domain_id', int, lambda v: 0 <= v <= 232, ' (0..232)'),
        gz_partition=_get(data, 'gz_partition', str, bool),
        gui=_get(data, 'simulation.gui', bool),
        rviz=_get(data, 'simulation.rviz', bool),
        start_timeout=_get(data, 'simulation.start_timeout', float, lambda v: v > 0),
        speed=_get(data, 'motion.speed', float, lambda v: 0.1 <= v <= 1.0, ' (0.1..1.0)'),
        tracking_tolerance=_get(data, 'motion.tracking_tolerance', float,
                                lambda v: 0.0 < v <= 0.5, ' (0..0.5 rad)'),
        first_gap=_get(data, 'live_control.first_gap', float, lambda v: 0.05 <= v <= 2.0,
                       ' (0.05..2 s)'),
        repeat_gap=_get(data, 'live_control.repeat_gap', float, lambda v: 0.05 <= v <= 1.0,
                        ' (0.05..1 s)'),
        arms=dict(arms),
        joints=joints,
        dof_pose={k: float(v) for k, v in dof_pose.items()},
        recordings_dir=Path(workspace) / _get(data, 'recordings.folder', str, bool),
        logs_dir=Path(workspace) / _get(data, 'logs.folder', str, bool),
    )
