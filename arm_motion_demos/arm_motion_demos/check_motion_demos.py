"""
Integration check of motion_demo against a running robot (mock or Gazebo).

    ros2 run arm_motion_demos check_motion_demos [--quick] [--skip-sequence]

Runs motion_demo as a subprocess for each scenario while recording
/joint_states itself, so the checks rest on the joint positions the robot
reports, not on the node's own exit code:
  - dry run; wave (both arms); dual_arm; dance67; the full sequence
  - invalid parameters are refused (exit 2) and nothing moves
  - Ctrl-C mid-gesture: the arms stop within the safe limits, then go home
  - a conflicting controller command mid-gesture is detected (exit 1, hold)
  - an obstacle in the wave's path makes the MoveIt collision check refuse
The robot moves (in simulation). Exits non-zero if any check fails.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

from arm_motion_demos import motions as m
from arm_motion_demos.motion_demo_node import load_robot
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LIMIT_SLACK = 0.005  # rad: actual positions may lag the plan by this much past the safe band


class Checker(Node):
    def __init__(self):
        super().__init__('check_motion_demos')
        self.limits, self.home = load_robot()
        self.latest = None
        self.samples = []
        self.results = []
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.arm_a = self.create_publisher(JointTrajectory, '/arm_a_controller/joint_trajectory',
                                           10)
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in m.JOINTS):
            self.latest = np.array([pos[j] for j in m.JOINTS])
            self.samples.append(self.latest)

    def report(self, ok, text):
        ok = bool(ok)
        self.results.append(ok)
        print(f'[{"PASS" if ok else "FAIL"}] {text}', flush=True)
        return ok

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_demo(self, params, timeout=600.0, during=None):
        """
        Run motion_demo with `params`; `during(proc)` is called while it runs.

        motion_demo runs in its own process group, so a Ctrl-C (SIGINT to the group)
        reaches the node itself and not only the `ros2 run` wrapper.

        Returns (exit code, recorded joint positions (N, 14), output).
        """
        args = ['ros2', 'run', 'arm_motion_demos', 'motion_demo', '--ros-args']
        for k, v in params.items():
            args += ['-p', f'{k}:={v}']
        self.samples = []
        proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, start_new_session=True)
        start = time.monotonic()
        while proc.poll() is None and time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if during:
                during(proc)
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
        out = proc.communicate()[0]
        self.spin_for(0.5)
        return proc.returncode, np.array(self.samples), out

    # --- checks on the recorded joint positions ----------------------------------
    def check_limits(self, label, q):
        """Inside the safe band, or between the starting pose and it (homing)."""
        lo, hi = self.limits.safe_lower - LIMIT_SLACK, self.limits.safe_upper + LIMIT_SLACK
        lo = np.maximum(np.minimum(lo, q[0]), self.limits.lower)
        hi = np.minimum(np.maximum(hi, q[0]), self.limits.upper)
        dist = np.minimum(q - self.limits.lower, self.limits.upper - q).min(axis=0)
        j = int(dist.argmin())
        return self.report(((q >= lo) & (q <= hi)).all(),
                           f'{label}: joints stayed inside the safe limits '
                           f'(closest {m.JOINTS[j]} {dist[j]:.4f} rad from its URDF limit)')

    def check_home(self, label):
        self.spin_for(0.5)
        err = np.abs(self.latest - self.home)
        j = int(err.argmax())
        return self.report(err.max() < 0.02, f'{label}: robot at home '
                                             f'(worst {m.JOINTS[j]} {err[j]:.4f} rad)')

    def check_moved(self, label, q, joint, at_least):
        span = float(np.ptp(q[:, m.INDEX[joint]])) if len(q) else 0.0
        return self.report(span >= at_least, f'{label}: {joint} moved {span:.2f} rad '
                                             f'(expected >= {at_least})')

    def check_still(self, label, q):
        span = float(np.ptp(q, axis=0).max()) if len(q) else 0.0
        return self.report(span < 0.005, f'{label}: robot did not move ({span:.4f} rad)')

    def gesture(self, label, params, moved):
        code, q, out = self.run_demo(params)
        ok = self.report(code == 0, f'{label}: motion_demo exit code {code}')
        if not ok:
            print(out[-2000:])
        if len(q):
            self.check_limits(label, q)
            for joint, amount in moved:
                self.check_moved(label, q, joint, amount)
        self.check_home(label)

    # --- scenarios ---------------------------------------------------------------
    def dry_run(self):
        code, q, out = self.run_demo({'dry_run': 'true'}, timeout=120)
        self.report(code == 0 and 'Collision check passed' in out,
                    f'dry run: limits + MoveIt collision check pass (exit {code})')
        self.check_still('dry run', q)

    def invalid(self):
        for params, text in (({'dance_tempo': '200.0', 'sequence': '[dance67]'}, 'tempo'),
                             ({'repetitions': '-1'}, 'repetitions'),
                             ({'sequence': '[moonwalk]'}, 'unknown gesture'),
                             ({'speed': '2.5', 'sequence': '[wave]'}, 'limit')):
            code, q, out = self.run_demo(params, timeout=60)
            self.report(code == 2 and text in out,
                        f'invalid {params}: refused with exit {code} ("{text}" in log)')
            self.check_still(f'invalid {params}', q)

    def interrupt(self):
        sent = []

        def ctrl_c(proc):
            # arm_a_joint3 only gets below -1.3 while the dance oscillates
            if not sent and self.latest[m.INDEX['arm_a_joint3']] < -1.3:
                os.killpg(proc.pid, signal.SIGINT)
                sent.append(True)
        code, q, out = self.run_demo({'sequence': '[dance67]', 'repetitions': '20'},
                                     during=ctrl_c)
        self.report(code == 130 and 'Holding position' in out,
                    f'Ctrl-C: motion_demo held and exited with {code}')
        if len(q):
            self.check_limits('Ctrl-C', q)
        self.spin_for(1.5)  # the hold decelerates within the limits; measured < 1.25 s
        before = np.array(self.latest)
        self.spin_for(3.0)
        drift = float(np.abs(self.latest - before).max())
        self.report(drift < 0.01, f'Ctrl-C: arms stopped (moved {drift:.4f} rad in the 3 s '
                                  f'after the stop)')
        self.report(np.abs(self.latest - self.home).max() > 0.05,
                    'Ctrl-C: arms stopped mid-gesture, not at home')
        code, q, _ = self.run_demo({'sequence': '[home]'})
        self.report(code == 0, f'after Ctrl-C: sequence [home] exit code {code}')
        if len(q):
            self.check_limits('after Ctrl-C', q)
        self.check_home('after Ctrl-C')

    def conflicting_command(self):
        sent = []

        def interfere(proc):
            # arm_a reached the waving pose: another command freezes it mid-wave
            if not sent and self.latest[m.INDEX['arm_a_joint1']] > 1.25:
                q = [float(self.latest[m.INDEX[j]]) for j in m.ARM_A]
                point = JointTrajectoryPoint(positions=q, velocities=[0.0] * 6)
                point.time_from_start.nanosec = 500_000_000
                self.arm_a.publish(JointTrajectory(joint_names=m.ARM_A, points=[point]))
                sent.append(True)
        code, q, out = self.run_demo({'sequence': '[wave]', 'repetitions': '6'},
                                     during=interfere)
        self.report(code == 1 and 'off the plan' in out,
                    f'conflicting command: detected, exit {code}')
        if len(q):
            self.check_limits('conflicting command', q)
        code, _, _ = self.run_demo({'sequence': '[home]'})
        self.report(code == 0, f'after conflicting command: home exit code {code}')
        self.check_home('after conflicting command')

    def obstacle(self):
        if not self.scene.wait_for_service(timeout_sec=5.0):
            return self.report(False, 'obstacle: /apply_planning_scene not available')
        box = CollisionObject(id='check_motion_demos_box')
        box.header.frame_id = 'world'
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.1])]
        box.primitive_poses = [Pose()]
        box.primitive_poses[0].position.x, box.primitive_poses[0].position.y = 0.12, 0.35
        box.primitive_poses[0].position.z = 0.8  # where arm_a's waving hand goes
        box.operation = CollisionObject.ADD

        def apply(obj):
            req = ApplyPlanningScene.Request(scene=PlanningScene(is_diff=True))
            req.scene.world.collision_objects = [obj]
            future = self.scene.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            return future.result() is not None and future.result().success
        if not self.report(apply(box), 'obstacle: box added in the wave path'):
            return
        try:
            code, q, out = self.run_demo({'sequence': '[wave]'}, timeout=120)
            self.report(code == 2 and 'collision' in out and 'check_motion_demos_box' in out,
                        f'obstacle: wave refused by the collision check (exit {code})')
            self.check_still('obstacle', q)
        finally:
            box.operation = CollisionObject.REMOVE
            self.report(apply(box), 'obstacle: box removed')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--quick', action='store_true',
                        help='one repetition per gesture, no sequence')
    parser.add_argument('--skip-sequence', action='store_true')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Checker()
    try:
        end = time.monotonic() + 15.0
        while node.latest is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.report(node.latest is not None, '/joint_states received'):
            reps = {'repetitions': '1'} if args.quick else {}
            node.dry_run()
            node.invalid()
            node.gesture('wave arm_a', {'sequence': '[wave]', **reps},
                         [('arm_a_joint1', 1.2), ('arm_a_joint3', 0.4)])
            node.gesture('wave arm_b', {'sequence': '[wave]', 'wave_arm': 'arm_b', **reps},
                         [('arm_b_joint1', 1.2), ('arm_b_joint3', 0.4)])
            node.gesture('dual_arm', {'sequence': '[dual_arm]', **reps},
                         [('arm_a_joint2', 2.5), ('arm_b_joint2', 2.5), ('arm_a_joint1', 1.2)])
            node.gesture('dance67', {'sequence': '[dance67]', **reps},
                         [('arm_a_joint3', 1.4), ('arm_b_joint3', 1.4)])
            if not (args.quick or args.skip_sequence):
                node.gesture('sequence', {'sequence': '[home, wave, dual_arm, dance67, home]'},
                             [('arm_a_joint1', 1.2), ('arm_b_joint3', 1.4)])
            node.interrupt()
            node.conflicting_command()
            node.obstacle()
    finally:
        passed, total = sum(node.results), len(node.results)
        print(f'\n{passed}/{total} checks passed', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if total and passed == total else 1)


if __name__ == '__main__':
    main()
