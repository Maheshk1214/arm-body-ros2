from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_motion_player'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'run.sh']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eileen',
    maintainer_email='eileenteoh10399@gmail.com',
    description='Menu-driven motion player for the moretea dual-arm robot in Gazebo',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'player = arm_motion_player.cli:main',
            'check_player = arm_motion_player.check_player:main',
        ],
    },
)
