from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'spot_mini_mini_real'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vats_intern',
    maintainer_email='chandan22102@iiitnr.edu.in',
    description='Spot Mini Mini real robot hardware interface',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'real_interface=spot_mini_mini_real.real_interface:main',
        ],
    },
)
