"""Gazebo simulation + MoveIt in one command.

Refuses to start if a controller_manager or move_group (e.g. the mock demo) is already
running, then starts the unchanged arm_body_description/gazebo.launch.py and attaches
MoveIt to its controllers via move_group_gazebo.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_launch = os.path.join(
        get_package_share_directory('arm_body_description'), 'launch', 'gazebo.launch.py')
    moveit_launch = os.path.join(
        get_package_share_directory('arm_body_moveit_config'), 'launch',
        'move_group_gazebo.launch.py')

    # Scoped: include arguments otherwise leak, and rviz:=false would also turn off MoveIt's RViz
    gazebo = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'self_collide': LaunchConfiguration('self_collide'),
                          'rviz': 'false'}.items())])  # MoveIt's RViz replaces the plain one
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={'self_collide': LaunchConfiguration('self_collide'),
                          'rviz': LaunchConfiguration('rviz'),
                          'rviz_config': LaunchConfiguration('rviz_config')}.items())

    precheck = Node(package='arm_body_moveit_config', executable='check_sim_controllers.py',
                    arguments=['--precheck'], output='screen')

    def on_precheck(event, _context):
        if event.returncode == 0:
            return [gazebo, moveit]
        return [LogInfo(msg='Another controller_manager/move_group is running; not starting.'),
                EmitEvent(event=Shutdown(reason='precheck failed'))]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo world file'),
        DeclareLaunchArgument('self_collide', default_value='false',
                              description='Let the arms collide with the body and each other'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start MoveIt RViz'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='RViz config file (default: config/moveit.rviz)'),
        precheck,
        RegisterEventHandler(OnProcessExit(target_action=precheck, on_exit=on_precheck)),
    ])
