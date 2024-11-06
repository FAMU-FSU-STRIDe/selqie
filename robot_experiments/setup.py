from setuptools import find_packages, setup

package_name = 'robot_experiments'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JTB',
    maintainer_email='jtylerboylan@outlook.com',
    description='Robot Experiments Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_tracking_orientation = robot_experiments.mpc_tracking_orientation:main',
            'mpc_tracking_position = robot_experiments.mpc_tracking_position:main',
        ],
    },
)
