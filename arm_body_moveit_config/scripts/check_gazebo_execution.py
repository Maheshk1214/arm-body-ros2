#!/usr/bin/env python3
"""End-to-end check of MoveIt -> Gazebo execution.

Run with Gazebo and move_group_gazebo.launch.py (or moveit_gazebo.launch.py) up:
    ros2 run arm_body_moveit_config check_gazebo_execution.py

1. /joint_states is live, on sim time, and has all 14 joints.
2. move_group's current robot state matches /joint_states, and the ground plane is in
   the planning scene.
3-5. Plan + execute small joint-space moves for arm_a, arm_b and both_arms through
     the MoveGroup action, then check the joints Gazebo reports reached the goal.
     (The Gazebo controllers report success without checking the goal, so this check
     reads /joint_states.) For both_arms, also check the two arms moved at the same time.
6. Go home, let it settle, hold it for 40 s of sim time while watching the near-limit joints
   for creep, then move away and back (holding a position exactly on a joint limit used to jam
   the arm in Gazebo).
7. Optionally open/close the grippers. Each move ends back at home.
8. Plan-only: goals exactly on the joint limits at 0 must not be planned onto the limit.
   gazebo_execution.yaml narrows MoveIt's copy of those limits by 5e-3 rad, and MoveIt clamps
   such a goal to the narrowed bound, so every planned point must stay >= 5e-3 rad inside.
Exits non-zero if any check fails. The robot moves in simulation.
"""
import argparse
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_A = [f'arm_a_joint{i}' for i in range(1, 7)]
ARM_B = [f'arm_b_joint{i}' for i in range(1, 7)]
GRIPPERS = ['arm_a_gripper_joint', 'arm_b_gripper_joint']
ALL_JOINTS = ARM_A + ARM_B + GRIPPERS

# Small moves away from home inside the joint limits (joint2 <= 0, joint5 >= 0; joint6 stays at
# its home value, 0.01 rad above its limit). Every joint is
# given: a free joint gets a random goal from OMPL. At home the arms hang ~2 cm above Gazebo's
# floor; this pose lifts the gripper to ~9 cm and the straight joint path never goes lower than
# home, so Gazebo's tracking lag does not drag the gripper along the floor.
SMALL_A = {'arm_a_joint1': 0.25, 'arm_a_joint2': -0.8, 'arm_a_joint3': 0.4,
           'arm_a_joint4': 0.0, 'arm_a_joint5': 0.4, 'arm_a_joint6': 0.01}
SMALL_B = {k.replace('arm_a', 'arm_b'): v for k, v in SMALL_A.items()}
# SRDF named states: joint2 (max 0), joint5, joint6 and the grippers (min 0) sit 0.01 rad inside
# their limits.
HOME_A = {'arm_a_joint1': 0.0, 'arm_a_joint2': -0.01, 'arm_a_joint3': 0.0,
          'arm_a_joint4': 0.0, 'arm_a_joint5': 0.01, 'arm_a_joint6': 0.01}
HOME_B = {k.replace('arm_a', 'arm_b'): v for k, v in HOME_A.items()}
GRIPPER_CLOSED = 0.01
# URDF limits at 0 and their side: +1 = upper limit (joint must stay < 0), -1 = lower limit
PLANNING_MARGIN = 0.005  # gazebo_execution.yaml
LIMIT_AT_ZERO = {f'arm_{a}_{j}': side for a in 'ab'
                 for j, side in (('joint2', +1), ('joint5', -1), ('joint6', -1),
                                 ('gripper_joint', -1))}


class Checker(Node):
    def __init__(self, tolerance, velocity_scaling):
        super().__init__('check_gazebo_execution')
        self.tolerance = tolerance
        self.velocity_scaling = velocity_scaling
        self.latest = None
        self.samples = []  # (sim time, {joint: position}) while recording
        self.recording = False
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 50)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.results = []

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest = (stamp, pos)
        if self.recording:
            self.samples.append((stamp, pos))

    def report(self, ok, text):
        ok = bool(ok)
        self.results.append(ok)
        print(f'[{"PASS" if ok else "FAIL"}] {text}', flush=True)
        return ok

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def check_joint_states(self):
        end = time.monotonic() + 15.0
        while self.latest is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.report(self.latest is not None, '/joint_states received'):
            return False
        t0, pos = self.latest
        self.spin_for(2.0)
        t1, _ = self.latest
        missing = [j for j in ALL_JOINTS if j not in pos]
        self.report(not missing, f'/joint_states has all {len(ALL_JOINTS)} joints'
                    + (f' (missing {missing})' if missing else ''))
        # On sim time the stamp starts near 0 after launch, nowhere near the wall clock
        self.report(t1 > t0 and abs(t1 - time.time()) > 1e6,
                    f'/joint_states is live and on sim time (stamp {t0:.2f} -> {t1:.2f} s)')
        return not missing

    def check_moveit_state(self):
        if not self.report(self.scene.wait_for_service(timeout_sec=10.0),
                           'move_group /get_planning_scene available'):
            return
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ROBOT_STATE
                                     | PlanningSceneComponents.WORLD_OBJECT_NAMES)
        end = time.monotonic() + 15.0  # add_ground_plane.py runs just after move_group starts
        while True:
            future = self.scene.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            scene = future.result().scene
            objects = [o.id for o in scene.world.collision_objects]
            if 'ground_plane' in objects or time.monotonic() > end:
                break
            self.spin_for(0.5)
        self.report('ground_plane' in objects,
                    f'ground_plane in planning scene (objects {objects})')
        js = scene.robot_state.joint_state
        moveit = dict(zip(js.name, js.position))
        _, gz = self.latest
        err = max(abs(moveit[j] - gz[j]) for j in ALL_JOINTS)
        self.report(err < 0.01,
                    f'MoveIt state matches Gazebo /joint_states (max diff {err:.4f} rad)')

    def plan_and_execute(self, group, goal, label, record=False, plan_only=False):
        if not self.move.wait_for_server(timeout_sec=10.0):
            return self.report(False, f'{label}: /move_action not available')
        g = MoveGroup.Goal()
        req = g.request
        req.group_name = group
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.velocity_scaling
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnect'
        c = Constraints()
        for j, v in goal.items():
            c.joint_constraints.append(JointConstraint(
                joint_name=j, position=float(v), tolerance_above=1e-3, tolerance_below=1e-3,
                weight=1.0))
        req.goal_constraints = [c]
        g.planning_options.plan_only = plan_only
        g.planning_options.replan = False

        self.samples, self.recording = [], record
        send = self.move.send_goal_async(g)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.recording = False
            return self.report(False, f'{label}: goal rejected by move_group')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=120.0)
        self.recording = False
        if result.result() is None:
            return self.report(False, f'{label}: no result within 120 s')
        res = result.result().result
        code = res.error_code.val
        plan = res.planned_trajectory.joint_trajectory
        if plan_only:
            return code, plan
        duration = 0.0
        if plan.points:
            d = plan.points[-1].time_from_start
            duration = d.sec + d.nanosec * 1e-9
        if not self.report(code == MoveItErrorCodes.SUCCESS,
                           f'{label}: plan + execute returned {code} '
                           f'(planned {duration:.2f} s sim time, {len(plan.joint_names)} joints)'):
            return False
        # Let the joints settle: up to 2 s of sim time (Gazebo may run slower than real time)
        t_end = self.latest[0] + 2.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            t, pos = self.latest
            err = {j: abs(pos[j] - v) for j, v in goal.items()}
            worst = max(err, key=err.get)
            if err[worst] < self.tolerance or t > t_end:
                break
        return self.report(err[worst] < self.tolerance,
                           f'{label}: Gazebo reached goal (worst {worst} {err[worst]:.4f} rad, '
                           f'tolerance {self.tolerance})')

    def check_plan_inside_limits(self, group, goal, label):
        """Plan only (nothing moves); no planned point may come within the margin of a limit."""
        out = self.plan_and_execute(group, goal, label, plan_only=True)
        if isinstance(out, bool):  # action unavailable / rejected / timed out, already reported
            return out
        code, plan = out
        if not self.report(code == MoveItErrorCodes.SUCCESS and plan.points,
                           f'{label}: plan returned {code} ({len(plan.points)} points)'):
            return False
        closest, worst = float('inf'), None
        for point in plan.points:
            for j, v in zip(plan.joint_names, point.positions):
                if j in LIMIT_AT_ZERO and -LIMIT_AT_ZERO[j] * v < closest:
                    closest, worst = -LIMIT_AT_ZERO[j] * v, j
        return self.report(closest >= PLANNING_MARGIN - 1e-6,
                           f'{label}: plan stays off the limits (closest {worst} {closest:.4f} '
                           f'rad, margin {PLANNING_MARGIN})')

    def hold(self, seconds, target, label, settle=5.0):
        """Without commanding anything, let the joints settle for `settle` s of sim time, then watch
        the near-limit joints for `seconds` more: they must not creep towards their limits."""
        t_end = self.latest[0] + settle
        while self.latest[0] < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        t_start, start = self.latest
        closest = {j: float('inf') for j in LIMIT_AT_ZERO}  # distance inside the limit
        while self.latest[0] < t_start + seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            _, pos = self.latest
            for j, side in LIMIT_AT_ZERO.items():
                closest[j] = min(closest[j], -side * pos[j])
        _, pos = self.latest
        creep = {j: -side * start[j] - closest[j] for j, side in LIMIT_AT_ZERO.items()}
        worst_creep = max(creep, key=creep.get)
        self.report(creep[worst_creep] < 1e-4,
                    f'{label}: held {self.latest[0] - t_start:.1f} s sim time, no creep towards '
                    f'the limits (worst {worst_creep} {creep[worst_creep]:.2e} rad)')
        err = {j: abs(pos[j] - v) for j, v in target.items()}
        worst_err = max(err, key=err.get)
        # Gazebo's position control settles within ~1e-3 rad; allow half the home-to-margin gap
        self.report(err[worst_err] < 2.5e-3,
                    f'{label}: still at home (worst {worst_err} {err[worst_err]:.2e} rad)')
        worst_limit = min(closest, key=closest.get)
        return self.report(closest[worst_limit] >= PLANNING_MARGIN,
                           f'{label}: stayed off the limits (closest {worst_limit} '
                           f'{closest[worst_limit]:.4f} rad from its limit)')

    def check_simultaneous(self, label):
        """Both arms must be moving during an overlapping time window."""
        if len(self.samples) < 3:
            return self.report(False, f'{label}: too few /joint_states samples recorded')
        start = self.samples[0][1]

        def window(joints):
            moving = [t for t, p in self.samples
                      if max(abs(p[j] - start[j]) for j in joints) > 0.02]
            return (min(moving), max(moving)) if moving else None
        a, b = window(ARM_A), window(ARM_B)
        ok = a is not None and b is not None and min(a[1], b[1]) - max(a[0], b[0]) > 0.0
        fmt = (lambda w: f'{w[0]:.2f}-{w[1]:.2f} s' if w else 'did not move')
        return self.report(ok, f'{label}: arms moved together (arm_a {fmt(a)}, arm_b {fmt(b)})')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='max |goal - Gazebo joint position| after execution, rad')
    parser.add_argument('--velocity-scaling', type=float, default=0.1,
                        help='velocity and acceleration scaling (RViz default: 0.1)')
    parser.add_argument('--skip-grippers', action='store_true')
    parser.add_argument('--hold', type=float, default=40.0,
                        help='seconds of sim time to hold home before moving away')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Checker(args.tolerance, args.velocity_scaling)
    try:
        if node.check_joint_states():
            node.check_moveit_state()
            node.plan_and_execute('both_arms', {**HOME_A, **HOME_B}, 'both_arms: start at home')
            node.plan_and_execute('arm_a', SMALL_A, 'arm_a: small move')
            node.plan_and_execute('arm_a', HOME_A, 'arm_a: back to home')
            node.plan_and_execute('arm_b', SMALL_B, 'arm_b: small move')
            node.plan_and_execute('arm_b', HOME_B, 'arm_b: back to home')
            if node.plan_and_execute('both_arms', {**SMALL_A, **SMALL_B},
                                     'both_arms: coordinated move', record=True):
                node.check_simultaneous('both_arms: coordinated move')
            if node.plan_and_execute('both_arms', {**HOME_A, **HOME_B}, 'both_arms: back to home'):
                node.hold(args.hold, {**HOME_A, **HOME_B}, 'both_arms: hold home')
            node.plan_and_execute('both_arms', {**SMALL_A, **SMALL_B},
                                  'both_arms: move away after hold')
            node.plan_and_execute('both_arms', {**HOME_A, **HOME_B},
                                  'both_arms: home after hold')
            if not args.skip_grippers:
                for grp, joint in (('gripper_a', GRIPPERS[0]), ('gripper_b', GRIPPERS[1])):
                    node.plan_and_execute(grp, {joint: 0.6}, f'{grp}: open to 0.6 rad')
                    node.plan_and_execute(grp, {joint: GRIPPER_CLOSED}, f'{grp}: close')
            # Goals exactly on the limits at 0 (the old all-zero home)
            node.check_plan_inside_limits('both_arms', {j: 0.0 for j in ARM_A + ARM_B},
                                          'both_arms: all-zero goal')
            if not args.skip_grippers:
                for grp, joint in (('gripper_a', GRIPPERS[0]), ('gripper_b', GRIPPERS[1])):
                    node.check_plan_inside_limits(grp, {joint: 0.0}, f'{grp}: goal 0')
    finally:
        passed, total = sum(node.results), len(node.results)
        print(f'\n{passed}/{total} checks passed', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if total and passed == total else 1)


if __name__ == '__main__':
    main()
