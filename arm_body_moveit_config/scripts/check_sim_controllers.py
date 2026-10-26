#!/usr/bin/env python3
"""Guard for running MoveIt against Gazebo.

--precheck : exit 0 only if no controller_manager or move_group is running yet
             (e.g. the mock demo), so Gazebo can be started without a conflict.
default    : wait until exactly one controller_manager exists, its hardware is
             gz_ros2_control (not mock_components), and all required controllers
             are active; exit 0 when ready, 1 on conflict or timeout.

Both modes also scan local processes, because a node that is still starting (or uses
different DDS transport settings) may not be visible in the ROS graph yet: any
move_group, or a ros2_control_node (mock hardware; Gazebo never runs one), on this
ROS_DOMAIN_ID counts as a conflict. Processes on other domains are ignored.
"""
import argparse
import os
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from rclpy.node import Node

REQUIRED = ['joint_state_broadcaster', 'arm_a_controller', 'arm_b_controller',
            'gripper_a_controller', 'gripper_b_controller']
# gz_ros2_control creates its hardware without pluginlib, so plugin_name is empty and only the
# <ros2_control name="GazeboSimSystem"> from arm_body.urdf.xacro identifies it.
GAZEBO_COMPONENT = 'GazeboSimSystem'


def is_gazebo(component):
    return (component.name == GAZEBO_COMPONENT and not component.plugin_name
            and not component.class_type)


def count_nodes(node, name):
    return sum(1 for n, ns in node.get_node_names_and_namespaces() if n == name and ns == '/')


def local_conflicts(executables):
    """Return [(pid, name)] of this user's processes running one of `executables` on our domain."""
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    found = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = f.read().split(b'\0')
            name = os.path.basename(argv[0].decode(errors='replace'))
            if name not in executables:
                continue
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = dict(kv.split(b'=', 1) for kv in f.read().split(b'\0') if b'=' in kv)
        except OSError:  # process exited, or belongs to another user
            continue
        if env.get(b'ROS_DOMAIN_ID', b'0').decode() == domain:
            found.append((int(pid), name))
    return found


def call(node, client, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    future = client.call_async(client.srv_type.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def precheck(node):
    time.sleep(2.0)  # let graph discovery settle
    rclpy.spin_once(node, timeout_sec=0.5)
    busy = {n: count_nodes(node, n) for n in ('controller_manager', 'move_group')}
    procs = local_conflicts({'move_group', 'ros2_control_node'})
    if any(busy.values()) or procs:
        node.get_logger().error(
            f'Already running on ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}: '
            f'graph {busy}, processes {procs}. '
            'Stop the mock demo (or other Gazebo/MoveIt session) first.')
        return 1
    node.get_logger().info('No controller_manager or move_group running; OK to start Gazebo.')
    return 0


def wait_ready(node, timeout):
    list_ctrl = node.create_client(ListControllers, '/controller_manager/list_controllers')
    list_hw = node.create_client(ListHardwareComponents,
                                 '/controller_manager/list_hardware_components')
    deadline = time.monotonic() + timeout
    last = ''
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        managers = count_nodes(node, 'controller_manager')
        if managers > 1:
            node.get_logger().error(
                f'{managers} controller_manager nodes found: mock and Gazebo hardware would '
                'both drive the robot. Refusing to start MoveIt.')
            return 1
        if count_nodes(node, 'move_group') or local_conflicts({'move_group'}):
            node.get_logger().error('A move_group is already running. Refusing to start another.')
            return 1
        mock = local_conflicts({'ros2_control_node'})
        if mock:
            node.get_logger().error(f'ros2_control_node (mock hardware) is running: {mock}. '
                                    'Refusing to start MoveIt against Gazebo.')
            return 1
        hw = call(node, list_hw) if managers == 1 else None
        if hw is not None:
            comps = hw.component
            if comps and not all(is_gazebo(c) for c in comps):
                found = [f'{c.name} ({c.plugin_name or c.class_type or "no plugin"})'
                         for c in comps]
                node.get_logger().error(
                    f'controller_manager hardware is {found}, expected only {GAZEBO_COMPONENT} '
                    'from gz_ros2_control (mock hardware?). Refusing to start MoveIt.')
                return 1
            ctrl = call(node, list_ctrl)
            if comps and ctrl is not None:
                states = {c.name: c.state for c in ctrl.controller}
                missing = [c for c in REQUIRED if states.get(c) != 'active']
                if not missing:
                    node.get_logger().info('Gazebo controllers active: ' + ', '.join(REQUIRED))
                    return 0
                msg = f'waiting for controllers: {missing}'
                if msg != last:
                    node.get_logger().info(msg)
                    last = msg
        time.sleep(0.5)
    node.get_logger().error(f'Timed out after {timeout:.0f}s waiting for Gazebo controllers.')
    return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--precheck', action='store_true')
    parser.add_argument('--timeout', type=float, default=120.0)
    args, _ = parser.parse_known_args()  # ignore --ros-args added by launch

    rclpy.init()
    node = Node('check_sim_controllers')
    try:
        code = precheck(node) if args.precheck else wait_ready(node, args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
