from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'spot_mini_mini_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vats_intern',
    maintainer_email='vats@todo.todo',
    description='Spot Mini Mini control node — BezierGait + IK + ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_commander = spot_mini_mini_control.spot_commander:main',
        ],
    },
)
