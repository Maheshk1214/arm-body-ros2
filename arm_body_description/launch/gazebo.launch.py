"""Spawn the robot in Gazebo Harmonic with ros2_control and start the controllers."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('arm_body_description')
    xacro_file = os.path.join(pkg, 'urdf', 'arm_body.urdf.xacro')
    self_collide = LaunchConfiguration('self_collide')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' self_collide:=', self_collide]), value_type=str)
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    # Lets Gazebo resolve package://arm_body_description/meshes/... paths
    resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 ', world], 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'arm_body'])

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    arm_ctrls = Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=['arm_a_controller', 'arm_b_controller',
                                'gripper_a_controller', 'gripper_b_controller'])

    rviz_node = Node(package='rviz2', executable='rviz2', output='screen',
                     arguments=['-d', os.path.join(pkg, 'rviz', 'display.rviz')],
                     parameters=[{'use_sim_time': True}],
                     condition=IfCondition(rviz))

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf',
                              description='Gazebo world file'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Also open RViz'),
        DeclareLaunchArgument('self_collide', default_value='false',
                              description='Let the arms collide with the body and each other'),
        resource_path,
        gz_sim,
        rsp,
        clock_bridge,
        spawn,
        # Start controllers only after the robot exists in Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm_ctrls])),
        rviz_node,
    ])
