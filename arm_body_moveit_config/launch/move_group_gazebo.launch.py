"""Attach MoveIt (move_group + RViz) to an already running Gazebo simulation.

Start Gazebo first:   ros2 launch arm_body_description gazebo.launch.py
Then:                 ros2 launch arm_body_moveit_config move_group_gazebo.launch.py

Starts no robot_state_publisher and no controller_manager: Gazebo's launch owns both.
move_group starts only after check_sim_controllers.py confirms a single Gazebo
controller_manager with all controllers active, so MoveIt cannot attach to mock hardware.
Gazebo's ground plane is added to the planning scene (ground_plane:=false to skip).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def launch_setup(context):
    self_collide = LaunchConfiguration('self_collide').perform(context)
    # Read now: the nodes below start later from an event handler, when the value may have changed
    start_rviz = IfCondition(LaunchConfiguration('rviz')).evaluate(context)
    # Same xacro and arguments Gazebo spawned, so move_group's model matches the simulated robot
    gazebo_xacro = os.path.join(get_package_share_directory('arm_body_description'),
                                'urdf', 'arm_body.urdf.xacro')
    moveit_config = (
        MoveItConfigsBuilder('arm_body', package_name='arm_body_moveit_config')
        .robot_description(file_path=gazebo_xacro, mappings={'self_collide': self_collide})
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    sim_time = {'use_sim_time': True}
    gazebo_params = os.path.join(moveit_config.package_path, 'config', 'gazebo_execution.yaml')
    # The main speed (default_*_scaling_factor in joint_limits.yaml) for RViz's planning panel:
    # moveit_config.joint_limits gives RViz the per-joint limits only, so the panel would
    # otherwise start from its own scaling and plan at a different speed than move_group.
    with open(os.path.join(moveit_config.package_path, 'config', 'joint_limits.yaml')) as f:
        limits = yaml.safe_load(f)
    main_speed = {f'robot_description_planning.{k}': float(limits[k])
                  for k in ('default_velocity_scaling_factor',
                            'default_acceleration_scaling_factor') if k in limits}

    wait_for_controllers = Node(
        package='arm_body_moveit_config', executable='check_sim_controllers.py',
        arguments=['--timeout', LaunchConfiguration('controller_timeout')], output='screen')

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[
            moveit_config.to_dict(),
            gazebo_params,
            {
                'publish_robot_description_semantic': True,
                'allow_trajectory_execution': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
                'monitor_dynamics': False,
            },
            sim_time,
        ])

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', LaunchConfiguration('rviz_config').perform(context)
                   or os.path.join(moveit_config.package_path, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            gazebo_params,
            main_speed,
            sim_time,
        ])

    ground_plane = Node(
        package='arm_body_moveit_config', executable='add_ground_plane.py', output='screen',
        condition=IfCondition(LaunchConfiguration('ground_plane')))

    def on_ready(event, _context):
        if event.returncode == 0:
            return [move_group, ground_plane] + ([rviz] if start_rviz else [])
        return [LogInfo(msg='Gazebo controllers not ready or conflicting hardware; '
                            'not starting MoveIt.'),
                EmitEvent(event=Shutdown(reason='check_sim_controllers failed'))]

    return [wait_for_controllers,
            RegisterEventHandler(OnProcessExit(target_action=wait_for_controllers,
                                               on_exit=on_ready))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('self_collide', default_value='false',
                              description='Must match the value Gazebo was launched with'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='RViz config file (default: config/moveit.rviz)'),
        DeclareLaunchArgument('ground_plane', default_value='true',
                              description="Add Gazebo's ground plane to the planning scene"),
        DeclareLaunchArgument('controller_timeout', default_value='120',
                              description='Seconds to wait for the Gazebo controllers'),
        OpaqueFunction(function=launch_setup),
    ])
