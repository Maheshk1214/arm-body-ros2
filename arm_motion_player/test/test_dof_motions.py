"""Single-joint demos: per-joint ranges, clipping, refusals, clearance."""
from pathlib import Path

from arm_motion_demos import geometry
from arm_motion_demos import motions as m
from arm_motion_player import dof_motions as d
from arm_motion_player.config import load_config
import numpy as np
import pytest

share = pytest.importorskip('ament_index_python.packages').get_package_share_directory
xacro = pytest.importorskip('xacro')
yaml = pytest.importorskip('yaml')
CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'player.yaml'


@pytest.fixture(scope='module')
def urdf():
    return xacro.process_file(share('arm_body_description') + '/urdf/arm_body.urdf.xacro').toxml()


@pytest.fixture(scope='module')
def limits(urdf):
    with open(share('arm_body_moveit_config') + '/config/joint_limits.yaml') as f:
        return m.parse_limits(urdf, yaml.safe_load(f))


@pytest.fixture(scope='module')
def home():
    with open(share('arm_body_moveit_config') + '/config/arm_body.srdf') as f:
        return m.parse_home(f.read())


@pytest.fixture(scope='module')
def config(tmp_path_factory):
    return load_config(CONFIG, tmp_path_factory.mktemp('ws'))


@pytest.fixture(scope='module')
def dof_pose(home, config):
    q = home.copy()
    for arm in ('left', 'right'):
        for short, value in config.dof_pose.items():
            q[m.INDEX[config.joint_name(arm, short)]] = value
    return q


def directions(sides):
    return sorted(s.label.split(':')[0] for s in sides)


# --- per-joint ranges ------------------------------------------------------------
def test_joints_on_a_limit_at_home_only_move_away_from_it(home, limits):
    assert directions(d.plan_sides(home, ['arm_a_joint2'], 0.6, limits)) == ['down']  # max 0
    for j in ('arm_a_joint5', 'arm_a_joint6', 'arm_a_gripper_joint'):         # min 0
        assert directions(d.plan_sides(home, [j], 0.6, limits)) == ['up'], j
    assert directions(d.plan_sides(home, ['arm_a_joint1'], 0.5, limits)) == ['down', 'up']


def test_swing_is_clipped_to_each_joints_own_safe_range(home, limits):
    start = home.copy()
    start[m.INDEX['arm_a_joint1']] = 1.4  # 0.17 rad below joint1's 1.5708 limit
    up = [s for s in d.plan_sides(start, ['arm_a_joint1'], 0.5, limits)
          if s.label.startswith('up')][0]
    assert up.target[m.INDEX['arm_a_joint1']] == pytest.approx(limits.safe_upper[0])
    assert up.swing == pytest.approx(limits.safe_upper[0] - 1.4)
    wrist = d.plan_sides(home, ['arm_a_joint6'], 3.0, limits)  # joint6 goes to 2*pi
    assert wrist[0].target[m.INDEX['arm_a_joint6']] == pytest.approx(0.01 + 3.0)


def test_only_the_selected_joint_moves(dof_pose, limits):
    j = m.INDEX['arm_b_joint4']
    for side in d.plan_sides(dof_pose, ['arm_b_joint4'], 0.5, limits):
        motion = d.side_motion(dof_pose, side, limits, 0.5)
        others = np.delete(motion.q, j, axis=1) - np.delete(dof_pose, j)
        assert np.abs(others).max() < 1e-12
        assert motion.q[-1] == pytest.approx(dof_pose)  # back where it started
        assert m.validate(motion, limits) == []


def test_both_arms_move_the_same_joint_together(dof_pose, limits):
    sides = d.plan_sides(dof_pose, ['arm_a_joint3', 'arm_b_joint3'], 0.5, limits)
    motion = d.combine(dof_pose, sides, limits, 0.5, 'both')
    a, b = motion.q[:, m.INDEX['arm_a_joint3']], motion.q[:, m.INDEX['arm_b_joint3']]
    assert a == pytest.approx(b) and np.ptp(a) == pytest.approx(1.0)
    assert m.validate(motion, limits) == []


def test_speed_scales_the_duration(dof_pose, limits):
    side = d.plan_sides(dof_pose, ['arm_a_joint2'], 0.6, limits)[0]
    slow = d.side_motion(dof_pose, side, limits, 0.25).duration
    fast = d.side_motion(dof_pose, side, limits, 1.0).duration
    assert slow == pytest.approx(4 * fast, rel=0.05)


# --- refusals --------------------------------------------------------------------
def test_joint_left_on_a_limit_is_refused(home, limits):
    zeros = np.zeros(len(m.JOINTS))  # e.g. left there by a topic command
    with pytest.raises(m.MotionError, match='return home first'):
        d.plan_sides(zeros, ['arm_a_joint2'], 0.5, limits)


@pytest.mark.parametrize('joints,amplitude,match', [
    (['arm_c_joint1'], 0.5, 'unknown joint'),
    (['arm_a_joint1'], 0.0, 'amplitude'),
    (['arm_a_joint1'], 5.0, 'amplitude'),
    (['arm_a_joint2'], 0.03, 'no room'),  # at home joint2 can only go down, and too little
])
def test_invalid_requests_are_refused(home, limits, joints, amplitude, match):
    with pytest.raises(m.MotionError, match=match):
        d.plan_sides(home, joints, amplitude, limits)


# --- geometry --------------------------------------------------------------------
def test_from_home_some_swings_hit_the_floor(home, limits, urdf):
    """Why the DOF demo pose exists: from home, joint5's only direction hits the floor."""
    robot = geometry.Robot(urdf, share('arm_body_description'))
    side = d.plan_sides(home, ['arm_a_joint5'], 0.6, limits)[0]
    motion = d.side_motion(home, side, limits, 1.0)
    floor = min(robot.clearances(dict(zip(m.JOINTS, q)))[0] for q in motion.q[::25])
    assert floor < 0.0


@pytest.mark.parametrize('arms', [['left'], ['right'], ['left', 'right']])
def test_every_swing_from_the_dof_pose_is_clear(config, dof_pose, limits, urdf, arms):
    robot = geometry.Robot(urdf, share('arm_body_description'))
    assert m.validate(m.Motion(np.zeros(1), dof_pose[None, :], []), limits) == []
    worst = np.full(4, np.inf)
    for info in config.joints:
        joints = [config.joint_name(a, info.short) for a in arms]
        for side in d.plan_sides(dof_pose, joints, info.amplitude, limits):
            motion = d.side_motion(dof_pose, side, limits, 1.0)
            for q in motion.q[::50]:
                worst = np.minimum(worst, robot.clearances(dict(zip(m.JOINTS, q))))
    floor, arm_arm, a_body, b_body = worst
    assert floor >= 0.05 and arm_arm >= 0.05 and min(a_body, b_body) >= 0.02, worst
