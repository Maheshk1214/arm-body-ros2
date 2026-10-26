"""Simulation manager: environment, launch command, domain check, readiness."""
import os
from pathlib import Path

from garage_motion_player import simulation_manager as sm
import pytest


def test_environment_is_local_and_on_its_own_domain(monkeypatch):
    env = sm.ros_environment(215, 'garage')
    assert env == {'ROS_DOMAIN_ID': '215', 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
                   'GZ_PARTITION': 'garage', 'GZ_IP': '127.0.0.1'}
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '30')
    sm.apply_environment(env)
    assert os.environ['ROS_DOMAIN_ID'] == '215' and 'ROS_LOCALHOST_ONLY' not in os.environ


def test_launch_command_reuses_the_existing_launch_file():
    world = Path('/w/demo_recording.sdf')
    gui = sm.launch_command(world, gui=True, rviz=False)
    assert gui[:4] == ['ros2', 'launch', 'arm_body_moveit_config', 'moveit_gazebo.launch.py']
    assert 'world:=/w/demo_recording.sdf' in gui and 'rviz:=false' in gui
    headless = sm.launch_command(world, gui=False, rviz=True)
    assert 'world:=-s --headless-rendering /w/demo_recording.sdf' in headless
    assert 'rviz:=true' in headless


def fake_process(proc, pid, env, cmd):
    d = proc / str(pid)
    d.mkdir()
    (d / 'environ').write_bytes(b'\0'.join(e.encode() for e in env) + b'\0')
    (d / 'cmdline').write_bytes(b'\0'.join(c.encode() for c in cmd) + b'\0')


def test_processes_on_the_domain_are_found(tmp_path):
    fake_process(tmp_path, 10, ['ROS_DOMAIN_ID=215'], ['gz', 'sim', 'server'])
    fake_process(tmp_path, 11, ['ROS_DOMAIN_ID=86'], ['ros2', 'launch', 'galbot'])
    fake_process(tmp_path, 12, ['ROS_DOMAIN_ID=215'], ['bash'])  # not ROS / Gazebo
    fake_process(tmp_path, 13, ['ROS_DOMAIN_ID=215'], ['/opt/ros/jazzy/lib/move_group'])
    (tmp_path / 'self').mkdir()
    found = sm.processes_on_domain(215, proc=tmp_path, exclude=(13,))
    assert [pid for pid, _ in found] == [10]


class FakeSim:
    def __init__(self, alive=True):
        self.alive = alive
        self.proc = type('P', (), {'returncode': 1})()
        self.log_path = Path('/tmp/sim.log')

    def running(self):
        return self.alive

    def log_tail(self):
        return 'gz: crashed'


def test_wait_ready_returns_when_nothing_is_missing():
    states = iter([['arm_a_controller', 'MoveIt'], ['MoveIt'], []])
    seen = []
    sm.wait_ready(lambda: next(states), 10, progress=seen.append, poll=lambda: None)
    assert seen == ['waiting for arm_a_controller, MoveIt', 'waiting for MoveIt']


def test_wait_ready_reports_missing_controllers_on_timeout():
    with pytest.raises(sm.SimulationError, match='still waiting for arm_b_controller'):
        sm.wait_ready(lambda: ['arm_b_controller'], 0.2, progress=lambda t: None,
                      poll=lambda: None)


def test_wait_ready_reports_a_crashed_simulation():
    with pytest.raises(sm.SimulationError, match='(?s)exited.*gz: crashed'):
        sm.wait_ready(lambda: ['MoveIt'], 10, progress=lambda t: None, poll=lambda: None,
                      sim=FakeSim(alive=False))
