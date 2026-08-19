import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rby1_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rainbow Robotics',
    maintainer_email='rainbow@rainbow-robotics.com',
    description='Navigation, SLAM, LiDAR fusion, and driver stream integration package for RBY1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulated_lidar = rby1_navigation.simulated_lidar:main',
            'lidar_merger = rby1_navigation.lidar_merger:main',
            'stream_manager = rby1_navigation.stream_manager:main',
        ],
    },
)
