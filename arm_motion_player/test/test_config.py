"""config/player.yaml loading and validation."""
from pathlib import Path

from arm_motion_player.config import ConfigError, load_config
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'player.yaml'


def write(tmp_path, data):
    path = tmp_path / 'player.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.fixture
def data():
    return yaml.safe_load(CONFIG.read_text())


def test_shipped_config_loads(tmp_path):
    config = load_config(CONFIG, tmp_path)
    assert config.ros_domain_id == 215
    assert 0.1 <= config.speed <= 1.0
    assert [j.short for j in config.joints] == ['joint1', 'joint2', 'joint3', 'joint4',
                                                'joint5', 'joint6', 'gripper']
    assert config.recordings_dir == tmp_path / 'recordings'
    assert config.motion_folders == [tmp_path / 'motions']
    assert config.first_gap > 0.5 and config.repeat_gap < config.first_gap  # 500 ms repeat delay


def test_joint_names_follow_the_arm_prefix(tmp_path):
    config = load_config(CONFIG, tmp_path)
    assert config.joint_name('left', 'joint3') == 'arm_a_joint3'
    assert config.joint_name('right', 'gripper') == 'arm_b_gripper_joint'


@pytest.mark.parametrize('change,match', [
    (lambda d: d.pop('ros_domain_id'), 'missing "ros_domain_id"'),
    (lambda d: d.update(ros_domain_id=999), 'ros_domain_id'),
    (lambda d: d.update(ros_domain_id='215'), 'must be a int'),
    (lambda d: d['motion'].update(speed=2.0), 'speed'),
    (lambda d: d['motion'].update(speed=True), 'speed'),
    (lambda d: d['simulation'].update(gui='yes'), 'gui'),
    (lambda d: d.update(arms={'left': 'arm_a'}), 'arms'),
    (lambda d: d['joints']['joint3'].update(amplitude=0), 'amplitude'),
    (lambda d: d['joints']['joint3'].pop('suffix'), 'suffix'),
    (lambda d: d.update(joints={}), 'empty'),
    (lambda d: d['dof_pose'].pop('joint5'), 'dof_pose'),
    (lambda d: d['dof_pose'].update(joint5='up'), 'dof_pose.joint5'),
    (lambda d: d['live_control'].update(first_gap=0.0), 'live_control.first_gap'),
    (lambda d: d.pop('live_control'), 'missing "live_control.first_gap"'),
    (lambda d: d['motions'].update(folders='motions'), 'motions.folders'),
    (lambda d: d['motions'].update(folders=['']), 'motions.folders'),
])
def test_invalid_settings_are_reported(tmp_path, data, change, match):
    change(data)
    with pytest.raises(ConfigError, match=match):
        load_config(write(tmp_path, data), tmp_path)


def test_missing_and_broken_files(tmp_path):
    with pytest.raises(ConfigError, match='not found'):
        load_config(tmp_path / 'nope.yaml', tmp_path)
    bad = tmp_path / 'bad.yaml'
    bad.write_text('speed: [1, 2\n')
    with pytest.raises(ConfigError, match='not valid YAML'):
        load_config(bad, tmp_path)
