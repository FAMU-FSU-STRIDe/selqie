from setuptools import find_packages, setup

package_name = 'robot_utils'

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
    description='Robot Utilities Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zero_motors = robot_utils.zero_motors:main',
            'idle_odrives = robot_utils.idle_odrives:main',
            'ready_odrives = robot_utils.ready_odrives:main',
            'zero_odrives = robot_utils.zero_odrives:main',
            'clear_odrive_errors = robot_utils.clear_odrive_errors:main',
        ],
    },
)
