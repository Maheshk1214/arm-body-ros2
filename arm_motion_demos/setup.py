from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_motion_demos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/worlds',
            ['worlds/demo_recording.sdf', 'worlds/aruco_reach.sdf']),
        ('share/' + package_name + '/motions', glob('motions/*.yaml')),
        ('share/' + package_name + '/launch',
            ['launch/aruco_reach_sim.launch.py', 'launch/aruco_tf_view.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/aruco_reach.rviz', 'rviz/aruco_tf_view.rviz']),
        ('share/' + package_name + '/models/aruco_tag_0',
            ['models/aruco_tag_0/model.config', 'models/aruco_tag_0/model.sdf']),
        ('share/' + package_name + '/models/aruco_tag_0/materials/textures',
            ['models/aruco_tag_0/materials/textures/aruco_4x4_50_id0.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eileen',
    maintainer_email='eileenteoh10399@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_motion_node = arm_motion_demos.arm_motion_node:main',
            'motion_demo = arm_motion_demos.motion_demo_node:main',
            'check_motion_demos = arm_motion_demos.check_motion_demos:main',
            'record_demo = arm_motion_demos.record_demo:main',
            'aruco_reach = arm_motion_demos.aruco_reach:main',
            'camera_stream = arm_motion_demos.camera_stream:main',
            'aruco_tf = arm_motion_demos.aruco_tf:main',
        ],
    },
)
