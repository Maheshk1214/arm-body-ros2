import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmMotionNode(Node):
    def __init__(self):
        super().__init__('arm_motion_node')

        self.arm_a_pub = self.create_publisher(
            JointTrajectory,
            '/arm_a_controller/joint_trajectory',
            10
        )

        self.arm_b_pub = self.create_publisher(
            JointTrajectory,
            '/arm_b_controller/joint_trajectory',
            10
        )

        self.joint_positions = {}
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10)

    def on_joint_state(self, msg):
        self.joint_positions.update(zip(msg.name, msg.position))

    def joint_names(self, prefix):
        return [f'{prefix}_joint{i}' for i in range(1, 7)]

    def move_arm(self, publisher, prefix, positions, duration=3):
        msg = JointTrajectory()

        msg.joint_names = self.joint_names(prefix)

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = duration

        msg.points.append(point)
        publisher.publish(msg)

    def wait_for_controllers(self, timeout=10.0):
        """Wait until both controllers subscribe and /joint_states arrives."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (self.arm_a_pub.get_subscription_count() > 0
                    and self.arm_b_pub.get_subscription_count() > 0
                    and self.joint_positions):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def wait_until_reached(self, positions, tolerance=0.02, timeout=20.0):
        """
        Wait until /joint_states shows both arms at `positions`.

        Waiting on the robot instead of a fixed sleep keeps the steps in
        order when Gazebo runs slower than real time, and catches a joint
        that does not arrive (the Gazebo controllers do not check goals).
        """
        targets = {}
        for prefix in ('arm_a', 'arm_b'):
            targets.update(zip(self.joint_names(prefix), positions))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(abs(self.joint_positions.get(j, float('inf')) - v)
                   < tolerance for j, v in targets.items()):
                return True
        worst = max(targets, key=lambda j: abs(
            self.joint_positions.get(j, float('inf')) - targets[j]))
        self.get_logger().error(
            f'Arms did not reach the target within {timeout:.0f} s '
            f'(worst {worst}: at {self.joint_positions.get(worst)}, '
            f'target {targets[worst]})')
        return False

    def move_both(self, positions):
        self.move_arm(self.arm_a_pub, 'arm_a', positions)
        self.move_arm(self.arm_b_pub, 'arm_b', positions)
        return self.wait_until_reached(positions)

    def run_demo(self):
        # joint2 (max 0), joint5 and joint6 (min 0) stay 0.01 rad inside
        # their limits, as in the SRDF home: holding a joint exactly on its
        # limit jams it in Gazebo.
        home = [0.0, -0.01, 0.0, 0.0, 0.01, 0.01]
        raised = [0.3, -1.2, 0.0, 0.0, 0.01, 0.01]

        self.get_logger().info('Moving both arms home')
        if not self.move_both(home):
            return False

        self.get_logger().info('Raising both arms')
        if not self.move_both(raised):
            return False

        self.get_logger().info('Returning home')
        return self.move_both(home)


def main():
    rclpy.init()
    node = ArmMotionNode()

    try:
        if not node.wait_for_controllers():
            node.get_logger().error(
                'arm_a/arm_b controllers or /joint_states not available')
        elif node.run_demo():
            node.get_logger().info('Demo finished')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
