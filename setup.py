import os
from glob import glob
from setuptools import setup

package_name = 'gap_follower'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
    ],
    zip_safe=True,
    maintainer='George Hany',
    maintainer_email='georgehany064@gmail.com',
    description='A ROS 2 package implementing the gap following algorithm for autonomous vehicle navigation using LiDAR data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "steering_speed_exe = gap_follower.steering_speed_control:main",
            "twist2ackermann_exe = gap_follower.twist2ackermann_converter_node:main",
        ],
    },
)
