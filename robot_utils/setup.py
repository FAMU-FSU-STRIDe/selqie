from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_utils'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'trajectories'), glob('trajectories/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JTB',
    maintainer_email='jtylerboylan@outlook.com',
    description='Robot Utilities Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'starq_util = robot_utils.starq_util:main',
            'unitree_a1_util = robot_utils.unitree_a1_util:main',
            'selqie_mujoco_util = robot_utils.selqie_mujoco_util:main',
            'imu_util = robot_utils.imu_util:main',
        ],
    },
)
