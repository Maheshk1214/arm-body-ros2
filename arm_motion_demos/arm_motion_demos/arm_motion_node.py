import time

import rclpy
from rclpy.node import Node
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

    def move_arm(self, publisher, prefix, positions, duration=3):
        msg = JointTrajectory()

        msg.joint_names = [
            f'{prefix}_joint{i}' for i in range(1, 7)
        ]

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = duration

        msg.points.append(point)
        publisher.publish(msg)

    def run_demo(self):
        home = [0.0] * 6
        raised = [0.3, -1.2, 0.0, 0.0, 0.0, 0.0]

        self.get_logger().info('Moving both arms home')
        self.move_arm(self.arm_a_pub, 'arm_a', home)
        self.move_arm(self.arm_b_pub, 'arm_b', home)
        time.sleep(4)

        self.get_logger().info('Raising both arms')
        self.move_arm(self.arm_a_pub, 'arm_a', raised)
        self.move_arm(self.arm_b_pub, 'arm_b', raised)
        time.sleep(4)

        self.get_logger().info('Returning home')
        self.move_arm(self.arm_a_pub, 'arm_a', home)
        self.move_arm(self.arm_b_pub, 'arm_b', home)


def main():
    rclpy.init()
    node = ArmMotionNode()

    try:
        # Allow time for the publishers to connect.
        time.sleep(2)
        node.run_demo()
        time.sleep(4)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
