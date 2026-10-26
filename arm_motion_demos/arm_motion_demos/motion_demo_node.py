r"""
Run a sequence of gestures (wave, dual_arm, dance67) on the arm controllers.

    ros2 run arm_motion_demos motion_demo --ros-args \\
        -p sequence:="[wave, dual_arm, dance67]" -p speed:=1.0

Before anything moves, every gesture is built from the SRDF home and checked
against the URDF position/velocity limits and MoveIt's acceleration limits
(motions.validate), then for collisions with MoveIt's /check_state_validity
(the planning scene includes Gazebo's floor). While a gesture runs,
/joint_states is compared with the plan; on a large deviation, a joint near a
limit, lost /joint_states or Ctrl-C, all controllers are told to hold position.

Exit codes: 0 done, 1 failed while moving, 2 refused before moving,
130 interrupted.
"""
import sys

from arm_motion_demos import motions as m
from arm_motion_demos.executor import (Aborted, HOME_TOLERANCE, load_robot,
                                       RobotExecutor)
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
        self.robot = None  # RobotExecutor, created once the limits are loaded

    def param(self, name):
        return self.get_parameter(name).value

    def hold(self, reason):
        if self.robot:
            self.robot.hold(reason)

    def run(self):
        params = {k: self.param(k) for k in ('speed', 'wave_arm', 'dance_tempo',
                                             'dance_amplitude')}
        if self.param('repetitions'):
            params['repetitions'] = self.param('repetitions')
        names = list(self.param('sequence'))
        try:
            limits, home = load_robot()
            self.robot = RobotExecutor(self, limits)
            sequence = m.build_sequence(names, home, limits, params)
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
                               tolerance=self.param('tracking_tolerance'))
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
