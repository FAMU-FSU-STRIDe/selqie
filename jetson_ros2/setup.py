from setuptools import find_packages, setup

package_name = 'jetson_ros2'

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
    description='Jetson ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gpio_node = jetson_ros2.gpio_node:main',
            'jetson_status_node = jetson_ros2.jetson_status_node:main',
        ],
    },
)
