"""
Gazebo + MoveIt with an ArUco tag in front of arm A, and a camera for arm A on ROS.

camera:=gazebo (default) bridges the simulated gripper camera. camera:=stream publishes a
real camera instead (camera_stream: stream_url, horizontal_fov, calibration_file), so a
real tag in front of the real camera is placed in front of the simulated gripper camera.
RViz shows the robot, planned paths, the camera image with detections and the tag/goal.
Then run aruco_reach in a second terminal (see README).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('arm_motion_demos')
    moveit_gazebo = os.path.join(get_package_share_directory('arm_body_moveit_config'),
                                 'launch', 'moveit_gazebo.launch.py')
    camera = LaunchConfiguration('camera')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_gazebo),
        launch_arguments={
            # '-s' runs Gazebo without its window (gz_args = '-r -v 3 ' + world)
            'world': [PythonExpression(["'' if '", LaunchConfiguration('gazebo_gui'),
                                        "' == 'true' else '-s '"]),
                      os.path.join(pkg, 'worlds', 'aruco_reach.sdf')],
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': os.path.join(pkg, 'rviz', 'aruco_reach.rviz')}.items())

    gazebo_camera = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/arm_a/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/arm_a/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", camera, "' == 'gazebo'"])))

    stream_camera = Node(
        package='arm_motion_demos', executable='camera_stream', output='screen',
        parameters=[{'source': ParameterValue(LaunchConfiguration('stream_url'), value_type=str),
                     'horizontal_fov': LaunchConfiguration('horizontal_fov'),
                     'calibration_file': LaunchConfiguration('calibration_file'),
                     'rotation': ParameterValue(LaunchConfiguration('rotation'),
                                                value_type=int)}],
        condition=IfCondition(PythonExpression(["'", camera, "' == 'stream'"])))

    image_view = Node(package='rqt_image_view', executable='rqt_image_view',
                      arguments=['/arm_a/camera/aruco_debug'],
                      condition=IfCondition(LaunchConfiguration('image_view')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start MoveIt RViz'),
        DeclareLaunchArgument('image_view', default_value='false',
                              description='Also show the annotated image in rqt_image_view'),
        DeclareLaunchArgument('gazebo_gui', default_value='false',
                              description='Open the Gazebo window (RViz shows everything)'),
        DeclareLaunchArgument('camera', default_value='gazebo',
                              description='gazebo: simulated camera; stream: real camera'),
        DeclareLaunchArgument('stream_url', default_value='http://192.168.4.1:81/stream',
                              description='camera:=stream source: MJPEG URL or device index'),
        DeclareLaunchArgument('horizontal_fov', default_value='0.96',
                              description='camera:=stream FOV estimate (rad) without calibration'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='camera:=stream ROS camera calibration YAML'),
        DeclareLaunchArgument('rotation', default_value='0',
                              description='camera:=stream: 0, 90, 180, 270 deg clockwise'),
        # model://aruco_tag_0 for the world file
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        sim,
        gazebo_camera,
        stream_camera,
        image_view,
    ])
