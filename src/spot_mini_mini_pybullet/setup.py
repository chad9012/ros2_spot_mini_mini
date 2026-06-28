from setuptools import find_packages, setup

package_name = 'spot_mini_mini_pybullet'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pybullet','spotmicro','spot_bullet'],
    zip_safe=True,
    maintainer='vats_intern',
    maintainer_email='chandan22102@iiitnr.edu.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'spot_pybullet_teleop = spot_mini_mini_pybullet.spot_pybullet_teleop:main',
        ],
    },
)
