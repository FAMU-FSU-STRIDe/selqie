from setuptools import find_packages, setup

package_name = 'selqie_ui'

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
    description='SELQIE UI Scripts',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'selqie_terminal = selqie_ui.selqie_terminal:main',
            'selqie_joint_publisher = selqie_ui.selqie_joint_publisher:main',
        ],
    },
)
