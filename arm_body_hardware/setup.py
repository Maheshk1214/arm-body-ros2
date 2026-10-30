from setuptools import find_packages, setup

package_name = 'arm_body_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/arms.yaml', 'config/moveit_controllers_real.yaml',
             'config/real_execution.yaml']),
        ('share/' + package_name + '/launch', ['launch/real_arm.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/real_arm.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mesh',
    maintainer_email='maheshkuna1214@gmail.com',
    description='Feetech STS bridge so MoveIt can drive the real arm',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'feetech_bridge = arm_body_hardware.bridge:main',
            'calibrate = arm_body_hardware.calibrate:main',
        ],
    },
)
