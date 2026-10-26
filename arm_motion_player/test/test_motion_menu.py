"""Motion library in the menu: parameter text, sequences, arm selection."""
from arm_motion_demos import motion_library as ml
from arm_motion_demos import motions as m
from arm_motion_player import motion_menu as mm
import pytest


@pytest.fixture(scope='module')
def library():
    lib, problems = ml.load_library([ml.builtin_folder()])
    assert problems == []
    return lib


NAMES = ['dance67', 'dual_arm', 'wave']


def test_parameter_text(library):
    assert mm.parse_values('', library['dance67']) == {}
    assert mm.parse_values('repetitions=4 tempo=40', library['dance67']) == {
        'repetitions': 4, 'tempo': 40.0}
    assert mm.parse_values('arm=arm_b, repetitions=2', library['wave']) == {
        'arm': 'arm_b', 'repetitions': 2}


@pytest.mark.parametrize('text,match', [
    ('tempo', 'name=value'),
    ('speed=2', 'no parameter "speed"'),
    ('tempo=fast', 'dance67 tempo must be a number'),
    ('tempo=500', 'dance67 tempo'),
    ('repetitions=2.5', 'dance67 repetitions'),
])
def test_bad_parameter_text_is_explained(library, text, match):
    with pytest.raises(m.MotionError, match=match):
        mm.parse_values(text, library['dance67'])


def test_wave_arm_must_be_one_of_the_arms(library):
    with pytest.raises(m.MotionError, match='wave arm must be one of arm_a, arm_b'):
        mm.parse_values('arm=left', library['wave'])


def test_the_selected_arm_goes_to_arm_choices(library):
    assert mm.arm_values(library['wave'], ['arm_b']) == {'arm': 'arm_b'}
    assert mm.arm_values(library['wave'], ['arm_a', 'arm_b']) == {}  # both: its default
    assert mm.arm_values(library['dance67'], ['arm_b']) == {}         # no arm choice


def test_library_listing(library):
    text = mm.render_library(NAMES, library, ['broken.yaml: not valid YAML'])
    assert '[1] 67 dance' in text and '[3] Wave' in text
    assert 'tempo=54.0 bpm (10..200)' in text and 'arm=arm_a (arm_a|arm_b)' in text
    assert '(skipped) broken.yaml' in text and '[B] Back' in text
