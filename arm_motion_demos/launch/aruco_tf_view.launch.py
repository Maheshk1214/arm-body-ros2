"""
RViz only: the robot held at aruco_reach's look pose, a real camera and the live ArUco TF.

No Gazebo, no MoveIt, nothing moves. Shows arm_a_camera_optical_frame on the gripper and
aruco_tag wherever the camera sees the tag, plus the annotated camera image.
Arguments: stream_url (MJPEG URL or device index, default 0), horizontal_fov,
calibration_file, rotation, marker_id, marker_size.
"""
import os

from ament_index_python.packages import get_package_share_directory
from arm_motion_demos.aruco_reach import LOOK_POSE
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('arm_motion_demos')
    xacro_file = os.path.join(get_package_share_directory('arm_body_description'),
                              'urdf', 'arm_body.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    look = {f'arm_a_joint{i + 1}': v for i, v in enumerate(LOOK_POSE)}
    # joints 2/5 have a limit at 0: start every other joint just inside its range
    look.update({'arm_b_joint2': -0.01, 'arm_b_joint5': 0.01, 'arm_b_joint6': 0.01,
                 'arm_a_gripper_joint': 0.01, 'arm_b_gripper_joint': 0.01})

    return LaunchDescription([
        DeclareLaunchArgument('stream_url', default_value='0',
                              description='Camera: MJPEG URL or device index'),
        DeclareLaunchArgument('horizontal_fov', default_value='0.96',
                              description='FOV estimate (rad) without calibration'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='ROS camera calibration YAML'),
        DeclareLaunchArgument('rotation', default_value='0',
                              description='Turn the image upright: 0, 90, 180, 270 deg clockwise'),
        DeclareLaunchArgument('marker_id', default_value='0',
                              description='ArUco id to track (DICT_4X4_50)'),
        DeclareLaunchArgument('marker_size', default_value='0.05',
                              description='Black square side of the tag (m)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'robot_description': robot_description, 'zeros': look}]),
        Node(package='arm_motion_demos', executable='camera_stream', output='screen',
             parameters=[{'source': ParameterValue(LaunchConfiguration('stream_url'),
                                                   value_type=str),
                          'horizontal_fov': LaunchConfiguration('horizontal_fov'),
                          'calibration_file': LaunchConfiguration('calibration_file'),
                          'rotation': ParameterValue(LaunchConfiguration('rotation'),
                                                     value_type=int)}]),
        Node(package='arm_motion_demos', executable='aruco_tf', output='screen',
             parameters=[{'marker_size': LaunchConfiguration('marker_size'),
                          'marker_id': ParameterValue(LaunchConfiguration('marker_id'),
                                                      value_type=int)}]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(pkg, 'rviz', 'aruco_tf_view.rviz')]),
    ])
