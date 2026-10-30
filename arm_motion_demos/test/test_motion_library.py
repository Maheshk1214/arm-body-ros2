"""YAML motion library: same trajectories as the old Python, file checks, discovery."""
import math
from pathlib import Path
import textwrap

from arm_motion_demos import motion_library as ml
from arm_motion_demos import motions as m
import legacy_gestures as legacy
import numpy as np
import pytest

share = pytest.importorskip('ament_index_python.packages').get_package_share_directory
xacro = pytest.importorskip('xacro')
yaml = pytest.importorskip('yaml')
SOURCE = Path(__file__).resolve().parents[1] / 'motions'


@pytest.fixture(scope='module')
def limits():
    urdf = xacro.process_file(share('arm_body_description') +
                              '/urdf/arm_body.urdf.xacro').toxml()
    with open(share('arm_body_moveit_config') + '/config/joint_limits.yaml') as f:
        return m.parse_limits(urdf, yaml.safe_load(f))


@pytest.fixture(scope='module')
def home():
    with open(share('arm_body_moveit_config') + '/config/arm_body.srdf') as f:
        return m.parse_home(f.read())


@pytest.fixture(scope='module')
def library(home, limits):
    lib, problems = ml.load_library([SOURCE], home, limits)
    assert problems == []
    return lib


# --- the YAML files are the old gestures, sample for sample ------------------------------
CASES = [
    ('wave', {}, lambda h, lim, s: legacy.wave(h, lim, speed=s)),
    ('wave', {'arm': 'arm_b', 'repetitions': 5},
     lambda h, lim, s: legacy.wave(h, lim, 5, s, 'arm_b')),
    ('wave', {'repetitions': 1}, lambda h, lim, s: legacy.wave(h, lim, 1, s)),
    ('dual_arm', {}, lambda h, lim, s: legacy.dual_arm(h, lim, speed=s)),
    ('dual_arm', {'repetitions': 4}, lambda h, lim, s: legacy.dual_arm(h, lim, 4, s)),
    ('dance67', {}, lambda h, lim, s: legacy.dance67(h, lim, speed=s)),
    ('dance67', {'tempo': 40.0, 'amplitude': 0.3, 'repetitions': 2},
     lambda h, lim, s: legacy.dance67(h, lim, 2, 40.0, 0.3, s)),
    ('dance67', {'tempo': 20.0, 'amplitude': 0.05, 'repetitions': 1},
     lambda h, lim, s: legacy.dance67(h, lim, 1, 20.0, 0.05, s)),
]


@pytest.mark.parametrize('speed', [0.3, 0.5, 1.0, 1.1])
@pytest.mark.parametrize('name,values,old', CASES)
def test_yaml_builds_exactly_the_old_python_gesture(library, home, limits, name, values, old,
                                                    speed):
    new, ref = library[name].build(home, limits, values, speed), old(home, limits, speed)
    assert new.q.shape == ref.q.shape
    assert np.array_equal(new.q, ref.q)  # bit for bit
    assert new.phases == ref.phases      # same labels at the same times
    assert new.name == ref.name


def test_installed_and_source_motions_are_the_same_files():
    installed = ml.builtin_folder()
    names = sorted(p.name for p in SOURCE.glob('*.yaml'))
    assert names == ['dance67.yaml', 'dance67_elbow.yaml', 'dual_arm.yaml', 'macarena.yaml',
                     'wave.yaml']
    if installed != SOURCE and installed.is_dir():
        for n in names:
            assert (installed / n).read_text() == (SOURCE / n).read_text(), \
                f'{n}: rebuild (colcon build) to install the current file'


def test_every_motion_has_a_title_description_and_documented_parameters(library):
    for spec in library.values():
        assert spec.title and spec.description
        for p in spec.parameters.values():
            assert p.hint, f'{spec.name}.{p.name} has no help text'
    tempo = library['dance67'].parameters['tempo']
    assert tempo.describe() == 'tempo=54.0 bpm (10..200)'
    assert library['wave'].parameters['arm'].describe() == 'arm=arm_a (arm_a|arm_b)'


def test_resolve_fills_in_defaults_and_checks_values(library):
    assert library['dance67'].resolve({'tempo': 40}) == {
        'repetitions': 6, 'tempo': 40.0, 'amplitude': 0.25}
    with pytest.raises(m.MotionError, match='dance67 tempo'):
        library['dance67'].resolve({'tempo': 5})


# --- writing motion files ------------------------------------------------------------------
def write(tmp_path, text, name='custom.yaml'):
    path = tmp_path / name
    path.write_text(textwrap.dedent(text))
    return path


GOOD = """
    name: nod
    title: Nod
    description: Both arms forward, then a nod of the wrists.
    parameters:
      nods: {type: int, default: 2, min: 1, max: 5, help: number of nods}
      depth: {type: float, default: 0.2, min: 0.05, max: 0.4, unit: rad, help: nod size}
    poses:
      forward: {both_arms: [0.0, -1.57, 0.0, 0.0, 1.57, 0.01], grippers: 0.3}
    steps:
      - move: forward
        label: forward
      - repeat: nods
        steps:
          - oscillate: {amplitude: {arm_a_joint5: depth, arm_b_joint5: depth / 2}, cycles: 1}
            label: "nod {i}/{nods}"
      - hold: 0.5
      - move: home
        label: back
"""


def test_a_new_motion_file_builds_and_validates(tmp_path, home, limits):
    spec = ml.load_motion(write(tmp_path, GOOD))
    motion = spec.build(home, limits, {'nods': 3})
    assert m.validate(motion, limits) == []
    assert [text for _, text in motion.phases] == ['forward', 'nod 1/3', 'nod 2/3', 'nod 3/3',
                                                   'back']
    i, j = m.INDEX['arm_a_joint5'], m.INDEX['arm_b_joint5']
    swing = np.ptp(motion.q[:, [i, j]], axis=0)
    assert swing[1] < swing[0]  # depth / 2 on arm_b
    assert motion.q[-1] == pytest.approx(home)
    grip = motion.q[:, m.INDEX['arm_a_gripper_joint']]
    assert grip.max() == pytest.approx(0.3)  # one value for both grippers


def test_library_discovers_files_and_skips_broken_ones(tmp_path, home, limits):
    write(tmp_path, GOOD, 'nod.yaml')
    write(tmp_path, 'name: [', 'broken.yaml')
    write(tmp_path, GOOD.replace('name: nod', 'name: wave'), 'copy.yaml')
    lib, problems = ml.load_library([SOURCE, tmp_path, tmp_path / 'missing'], home, limits)
    assert sorted(lib) == ['dance67', 'dance67_elbow', 'dual_arm', 'macarena', 'nod', 'wave']
    assert lib['wave'].path.parent == SOURCE  # the first folder wins
    assert len(problems) == 2
    assert any('broken.yaml: not valid YAML' in p for p in problems)
    assert any("name 'wave' is already used" in p for p in problems)


def test_a_file_breaking_the_limits_is_skipped_with_the_reason(tmp_path, home, limits):
    fast = GOOD.replace('- move: forward\n        label: forward',
                        '- move: forward\n        label: forward\n        duration: 0.2')
    write(tmp_path, fast, 'fast.yaml')
    lib, problems = ml.load_library([tmp_path], home, limits)
    assert lib == {} and 'breaks the limits with its defaults' in problems[0]
    assert 'acceleration' in problems[0]


@pytest.mark.parametrize('change,match', [
    (lambda s: s.replace('title: Nod', 'colour: red'), 'unknown key.*colour'),
    (lambda s: s.replace('name: nod', 'name: home'), 'reserved'),
    (lambda s: s.replace('name: nod', 'name: no-d'), 'letters, digits'),
    (lambda s: s.replace('type: int', 'type: integer'), 'type must be int, float or choice'),
    (lambda s: s.replace('default: 2,', 'default: 9,'), 'default is invalid'),
    (lambda s: s.replace('min: 1, max: 5', 'min: 5, max: 1'), 'min is above max'),
    (lambda s: s.replace('min: 1, max: 5', 'min: 1.5, max: 5'), 'whole numbers'),
    (lambda s: s.replace('- move: forward', '- move: sideways'), "unknown pose 'sideways'"),
    (lambda s: s.replace('- hold: 0.5', '- hold: 0.5\n        move: home'), 'exactly one of'),
    (lambda s: s.replace('- hold: 0.5', '- wait: 0.5'), 'exactly one of'),
    (lambda s: s.replace('cycles: 1}', 'cycle: 1}'), 'unknown key.*cycle'),
    (lambda s: s.replace(', cycles: 1}', '}'), 'missing "cycles"'),
    (lambda s: s.replace('      - move: home\n        label: back\n', ''), 'end at home'),
])
def test_invalid_motion_files_are_explained(tmp_path, home, limits, change, match):
    with pytest.raises(m.MotionError, match=match):
        ml.load_motion(write(tmp_path, change(GOOD))).build(home, limits)


@pytest.mark.parametrize('change,match', [
    (lambda s: s.replace('[0.0, -1.57, 0.0, 0.0, 1.57, 0.01]', '[0.0, -1.57]'), 'needs 6'),
    (lambda s: s.replace('both_arms:', 'both_legs:'), 'unknown joint or group'),
    (lambda s: s.replace('arm_a_joint5: depth', 'arm_a_joint5: dpeth'), "unknown name 'dpeth'"),
    (lambda s: s.replace('arm_a_joint5: depth', 'arm_a_joint5: __import__("os")'),
     'not a supported expression'),
    (lambda s: s.replace('arm_a_joint5: depth', 'arm_a_joint5: depth / 0'), 'division'),
    (lambda s: s.replace('"nod {i}/{nods}"', '"nod {j}"'), 'cannot fill in'),
    (lambda s: s.replace('- repeat: nods', '- repeat: 0'), 'whole number'),
    (lambda s: s.replace('- hold: 0.5', '- hold: -1'), 'must be positive'),
])
def test_invalid_values_are_explained_with_their_place(tmp_path, home, limits, change, match):
    with pytest.raises(m.MotionError, match=match):
        ml.load_motion(write(tmp_path, change(GOOD))).build(home, limits)


def test_errors_name_the_file_and_the_place(tmp_path, home, limits):
    bad = GOOD.replace('arm_a_joint5: depth', 'arm_a_joint5: dpeth')
    with pytest.raises(m.MotionError) as e:
        ml.load_motion(write(tmp_path, bad, 'nod.yaml')).build(home, limits)
    assert str(e.value).startswith('nod.yaml: steps[1].steps[0].oscillate.amplitude')


@pytest.mark.parametrize('text,expected', [
    ('2 * 60 / tempo', 2 * 60 / 54.0), ('-amplitude', -0.25), ('pi / 2', math.pi / 2),
    ('max(1, tempo) ** 0.5', math.sqrt(54.0)), ('abs(-3)', 3.0), (1.5, 1.5), (2, 2.0),
])
def test_expressions(text, expected):
    assert ml.evaluate(text, {'tempo': 54.0, 'amplitude': 0.25}, 'x') == pytest.approx(expected)


@pytest.mark.parametrize('text', ['tempo.real', 'open("x")', '[1]', 'True', '1 if 1 else 2',
                                  'lambda: 1', '10 ** 400'])
def test_unsupported_expressions_are_refused(text):
    with pytest.raises(m.MotionError):
        ml.evaluate(text, {'tempo': 54.0}, 'x')


def test_choice_parameters_fill_in_joint_names(tmp_path, home, limits):
    text = """
        parameters:
          side: {type: choice, default: arm_b, choices: [arm_a, arm_b], help: which arm}
        poses:
          out: {"{side}_joint1": 0.5}
        steps:
          - move: out
          - move: home
    """
    spec = ml.load_motion(write(tmp_path, text, 'reach.yaml'))
    assert spec.name == 'reach'  # from the file name
    for side in ('arm_a', 'arm_b'):
        q = spec.build(home, limits, {'side': side}).q
        assert q[:, m.INDEX[f'{side}_joint1']].max() == pytest.approx(0.5)
