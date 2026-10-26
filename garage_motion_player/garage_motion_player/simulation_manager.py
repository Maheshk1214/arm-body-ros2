"""
Start, check and stop the existing Gazebo + controllers + MoveIt simulation.

Launches arm_body_moveit_config/moveit_gazebo.launch.py (unchanged) in its own
process group with arm_motion_demos' camera world, so the same session can
record videos. Readiness is checked, not waited for: the caller passes a
function listing what is still missing (controllers, /joint_states, MoveIt).
"""
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Callable, Dict, List, Optional, Tuple


def ros_environment(domain: int, gz_partition: str) -> Dict[str, str]:
    """
    Environment for this robot only: its own domain, localhost-only traffic.

    Localhost-only keeps a Wi-Fi drop from stalling ROS and Gazebo transport.
    """
    return {'ROS_DOMAIN_ID': str(domain), 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
            'GZ_PARTITION': gz_partition, 'GZ_IP': '127.0.0.1'}


def apply_environment(env: Dict[str, str]) -> None:
    """Set `env` for this process and its children (before rclpy.init)."""
    os.environ.update(env)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)  # deprecated; conflicts with the range


def processes_on_domain(domain: int, proc: Path = Path('/proc'),
                        exclude: Tuple[int, ...] = ()) -> List[Tuple[int, str]]:
    """ROS/Gazebo processes whose environment has ROS_DOMAIN_ID=`domain`."""
    wanted = f'ROS_DOMAIN_ID={domain}'.encode()
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) in exclude:
            continue
        try:
            env = (entry / 'environ').read_bytes().split(b'\0')
            cmd = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except OSError:  # gone, or not ours
            continue
        if wanted in env and any(k in cmd for k in ('ros', 'gz sim', 'gazebo', 'rviz')):
            found.append((int(entry.name), cmd.strip()[:100]))
    return found


def world_argument(world: Path, gui: bool) -> str:
    """Gazebo world argument: with the GUI, or server-only with offscreen rendering."""
    return str(world) if gui else f'-s --headless-rendering {world}'


def launch_command(world: Path, gui: bool, rviz: bool) -> List[str]:
    return ['ros2', 'launch', 'arm_body_moveit_config', 'moveit_gazebo.launch.py',
            f'world:={world_argument(world, gui)}', f'rviz:={str(rviz).lower()}']


class SimulationError(RuntimeError):
    """The simulation could not be started or did not become ready."""


class Simulation:
    def __init__(self, world: Path, gui: bool, rviz: bool, log_dir: Path):
        self.command = launch_command(world, gui, rviz)
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f'simulation_{time.strftime("%Y%m%d_%H%M%S")}.log'
        self.proc: Optional[subprocess.Popen] = None

    def start(self) -> None:
        self._log = open(self.log_path, 'w')
        self.proc = subprocess.Popen(self.command, stdout=self._log, stderr=subprocess.STDOUT,
                                     start_new_session=True)  # own group: stop() reaches all

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def log_tail(self, lines: int = 15) -> str:
        try:
            return '\n'.join(self.log_path.read_text(errors='replace').splitlines()[-lines:])
        except OSError:
            return ''

    def stop(self, timeout: float = 20.0) -> None:
        """Ctrl-C the whole launch, then terminate/kill whatever is left."""
        if self.proc is None:
            return
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5.0),
                          (signal.SIGKILL, 5.0)):
            if not self._group_alive():
                break
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            end = time.monotonic() + wait
            while self._group_alive() and time.monotonic() < end:
                self.proc.poll()  # reap the launch process, or its zombie keeps the group
                time.sleep(0.2)
        self.proc.poll()
        self._log.close()

    def _group_alive(self) -> bool:
        try:
            os.killpg(self.proc.pid, 0)
            return True
        except ProcessLookupError:
            return False


def wait_ready(missing: Callable[[], List[str]], timeout: float,
               progress: Callable[[str], None], poll: Callable[[], None],
               sim: Optional[Simulation] = None) -> None:
    """Return once missing() is empty; raise SimulationError if `sim` exits or on timeout."""
    end, last = time.monotonic() + timeout, None
    while True:
        poll()
        still = missing()
        if not still:
            return
        if sim is not None and not sim.running():
            raise SimulationError(f'the simulation exited (code {sim.proc.returncode}). '
                                  f'Last lines of {sim.log_path}:\n{sim.log_tail()}')
        if time.monotonic() > end:
            where = f' Log: {sim.log_path}' if sim else ''
            raise SimulationError(f'not ready after {timeout:.0f} s, still waiting for '
                                  f'{", ".join(still)}.{where}')
        text = 'waiting for ' + ', '.join(still)
        if text != last:
            progress(text)
            last = text
