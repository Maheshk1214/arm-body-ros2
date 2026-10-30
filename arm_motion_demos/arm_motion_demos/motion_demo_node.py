r"""
Run a sequence of motions from the YAML library on the arm controllers.

    ros2 run arm_motion_demos motion_demo --ros-args \\
        -p sequence:="[wave, dual_arm, dance67]" -p speed:=1.0

The motions are the *.yaml files in this package's motions/ folder (and in
the folders of the motion_folders parameter). Before anything moves, every
motion is built from the SRDF home and checked
against the URDF position/velocity limits and MoveIt's acceleration limits
(motions.validate), then for collisions with MoveIt's /check_state_validity
(the planning scene includes Gazebo's floor). While a gesture runs,
/joint_states is compared with the plan; on a large deviation, a joint near a
limit, lost /joint_states or Ctrl-C, all controllers are told to hold position.

Exit codes: 0 done, 1 failed while moving, 2 refused before moving,
130 interrupted.
"""
from pathlib import Path
import sys

from arm_motion_demos import motion_library as ml
from arm_motion_demos import motions as m
from arm_motion_demos.executor import (Aborted, GOAL_TOLERANCE, HOME_TOLERANCE,
                                       LIMIT_GUARD, load_robot, RobotExecutor)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class MotionDemo(Node):
    def __init__(self):
        super().__init__('motion_demo')
        self.declare_parameter('sequence', ['wave', 'dual_arm', 'dance67'])
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('repetitions', 0)        # 0 = each gesture's default
        self.declare_parameter('wave_arm', 'arm_a')
        self.declare_parameter('dance_tempo', 54.0)     # beats/min, sim time
        self.declare_parameter('dance_amplitude', 0.25)  # rad of forearm tilt
        self.declare_parameter('collision_check', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('tracking_tolerance', 0.2)  # rad, plan vs /joint_states
        self.declare_parameter('goal_tolerance', GOAL_TOLERANCE)  # rad, final pose reached
        self.declare_parameter('limit_guard', LIMIT_GUARD)  # rad, stop this close to a limit
        self.declare_parameter('motion_folders', [''])     # extra folders of *.yaml motions
        self.robot = None  # RobotExecutor, created once the limits are loaded

    def param(self, name):
        return self.get_parameter(name).value

    def hold(self, reason):
        if self.robot:
            self.robot.hold(reason)

    def run(self):
        # the original parameter names, mapped onto the motions' own parameters
        values = {'wave': {'arm': self.param('wave_arm')},
                  'dance67': {'tempo': self.param('dance_tempo'),
                              'amplitude': self.param('dance_amplitude')},
                  'dance67_elbow': {'tempo': self.param('dance_tempo'),
                                    'amplitude': self.param('dance_amplitude')}}
        common = {'repetitions': self.param('repetitions')} if self.param('repetitions') else {}
        names = list(self.param('sequence'))
        folders = [ml.builtin_folder()] + [Path(f) for f in self.param('motion_folders') if f]
        try:
            limits, home = load_robot()
            self.robot = RobotExecutor(self, limits)
            library, problems = ml.load_library(folders)
            for problem in problems:
                self.get_logger().warn(f'Motion file skipped: {problem}')
            values = {k: v for k, v in values.items() if k in names}
            sequence = ml.build_sequence(library, names, home, limits, values,
                                         self.param('speed'), common)
        except m.MotionError as e:
            self.get_logger().error(f'Refusing to move: {e}')
            return 2
        total = sum(mo.duration for _, mo in sequence)
        self.get_logger().info(
            f'Sequence: home -> {" -> ".join(n for n, _ in sequence)} -> home '
            f'({total:.0f} s of controller time); all limit checks passed')
        missing = self.robot.wait_for_robot()
        if missing:
            self.get_logger().error(f'Robot not available: {", ".join(missing)} missing')
            return 2
        if self.param('collision_check'):
            if not self.robot.validity.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(
                    'MoveIt /check_state_validity not available: start move_group '
                    '(demo.launch.py or moveit_gazebo.launch.py) or set collision_check:=false')
                return 2
            for name, motion in sequence:
                problem = self.robot.check_collisions(name, motion)
                if problem:
                    self.get_logger().error(f'Refusing to move: {problem}')
                    return 2
            self.get_logger().info('Collision check passed (MoveIt planning scene)')
        else:
            self.get_logger().warn('Collision check disabled')
        if self.param('dry_run'):
            self.get_logger().info('Dry run: not moving')
            return 0
        _, q, _ = self.robot.state
        if self.robot.is_moving():
            self.get_logger().error('Robot is moving (is MoveIt executing?); not starting')
            return 2
        if np.abs(q - home).max() > HOME_TOLERANCE:
            homing = m.home_motion(q, home, limits, self.param('speed'))
            problems = m.validate(homing, limits, allow_start=True)
            if problems:
                self.get_logger().error('Cannot go home safely: ' + '; '.join(problems))
                return 2
            if self.param('collision_check'):
                problem = self.robot.check_collisions('home', homing)
                if problem:
                    self.get_logger().error(f'Refusing to move: {problem}')
                    return 2
            sequence.insert(0, ('home', homing))
        for name, motion in sequence:
            self.robot.execute(name, motion, guard_limits=name != 'home',
                               tolerance=self.param('tracking_tolerance'),
                               goal_tolerance=self.param('goal_tolerance'),
                               limit_guard=self.param('limit_guard'))
        self.get_logger().info('Sequence finished at home')
        return 0


def main():
    # Python's own SIGINT handling, so the hold command can still be published
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MotionDemo()
    code = 1
    try:
        code = node.run()
    except Aborted as e:
        node.get_logger().error(f'Stopped: {e}')
        code = 1
    except KeyboardInterrupt:
        node.hold('interrupted (Ctrl-C)')
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
