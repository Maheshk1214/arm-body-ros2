"""Loading the arm config, the calibration and the URDF joint limits."""
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

from arm_body_hardware.feetech import TICKS_PER_REV

RAD_PER_TICK = 2 * math.pi / TICKS_PER_REV


def default_config_path():
    return os.path.join(get_package_share_directory('arm_body_hardware'), 'config', 'arm_a.yaml')


def load_config(path=None):
    with open(path or default_config_path()) as f:
        cfg = yaml.safe_load(f)
    cfg['calibration_file'] = os.path.expanduser(cfg['calibration_file'])
    return cfg


def load_calibration(cfg):
    """{joint: zero_raw}; raises FileNotFoundError if the arm was never calibrated."""
    with open(cfg['calibration_file']) as f:
        return yaml.safe_load(f)['zero_raw']


def save_calibration(cfg, zero_raw):
    os.makedirs(os.path.dirname(cfg['calibration_file']), exist_ok=True)
    with open(cfg['calibration_file'], 'w') as f:
        f.write('# Raw servo position at the URDF zero pose. Written by arm_body_hardware calibrate.\n')
        yaml.safe_dump({'zero_raw': zero_raw}, f)


def urdf_limits():
    """{joint: (lower, upper)} for the revolute joints of the MoveIt robot description."""
    import xacro
    path = os.path.join(get_package_share_directory('arm_body_moveit_config'),
                        'config', 'arm_body.urdf.xacro')
    root = ET.fromstring(xacro.process_file(path).toxml())
    limits = {}
    for joint in root.iter('joint'):
        lim = joint.find('limit')
        if joint.get('type') == 'revolute' and lim is not None:
            limits[joint.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    return limits


def to_rad(raw, zero_raw, sign):
    return sign * (raw - zero_raw) * RAD_PER_TICK


def to_raw(rad, zero_raw, sign):
    return zero_raw + sign * rad / RAD_PER_TICK
