"""MoveIt + RViz driving the real arms A and B, one Feetech bus each.

    ros2 launch arm_body_hardware real_arm.launch.py monitor:=true  # torque off, RViz mirrors
    ros2 launch arm_body_hardware real_arm.launch.py                # MoveIt drives the arms
    ros2 launch arm_body_hardware real_arm.launch.py arms:=arm_a    # only arm A real

Run `ros2 run arm_body_hardware calibrate --arm <arm>` once per arm first. Do not run alongside
the mock demo or Gazebo: they publish /joint_states and controllers with the same names.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    hw_share = get_package_share_directory('arm_body_hardware')
    moveit_config = (
        MoveItConfigsBuilder('arm_body', package_name='arm_body_moveit_config')
        .trajectory_execution(
            file_path=os.path.join(hw_share, 'config', 'moveit_controllers_real.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    real_params = os.path.join(hw_share, 'config', 'real_execution.yaml')

    bridge = Node(
        package='arm_body_hardware', executable='feetech_bridge', output='screen',
        parameters=[{'config': LaunchConfiguration('config'),
                     'monitor': LaunchConfiguration('monitor'),
                     'arms': ParameterValue(LaunchConfiguration('arms'), value_type=str)}])

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='log',
        parameters=[moveit_config.robot_description])

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[
            moveit_config.to_dict(),
            real_params,
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            },
        ])

    # Gripper camera (USB) on /arm_a/camera/image_raw, framed at arm_a_camera_optical_frame
    camera = Node(
        package='arm_motion_demos', executable='camera_stream', output='screen',
        condition=IfCondition(LaunchConfiguration('camera')),
        parameters=[{'source': ParameterValue(LaunchConfiguration('camera_device'),
                                              value_type=str),
                     'fourcc': 'MJPG'}])

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(hw_share, 'rviz', 'real_arm.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            real_params,
        ])

    return LaunchDescription([
        DeclareLaunchArgument('monitor', default_value='false',
                              description='Torque off; RViz only mirrors the arm moved by hand'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('config', default_value='',
                              description='Arms config yaml (default: config/arms.yaml)'),
        DeclareLaunchArgument('arms', default_value='',
                              description='Real arms, comma-separated (default: all)'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the gripper camera (camera_stream)'),
        DeclareLaunchArgument('camera_device', default_value='0',
                              description='Gripper camera device index (/dev/videoN)'),
        bridge, rsp, move_group, camera, rviz,
    ])
