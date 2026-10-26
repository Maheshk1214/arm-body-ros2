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
        ('share/' + package_name + '/worlds', ['worlds/demo_recording.sdf']),
        ('share/' + package_name + '/motions', glob('motions/*.yaml')),
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
        ],
    },
)
