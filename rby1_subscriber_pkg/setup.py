from setuptools import setup
import os
from glob import glob

package_name = 'rby1_subscriber_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iaunoh@rainbow-robotics.com',
    maintainer_email='iaunoh@rainbow-robotics.com',
    description='Simple subscriber package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '01_dualarm_control = rby1_subscriber_pkg.01_dualarm_control:main',
            '02_wholebody_control = rby1_subscriber_pkg.02_wholebody_control:main',
            '03_torso_control = rby1_subscriber_pkg.03_torso_control:main',
            '04_right_arm_control = rby1_subscriber_pkg.04_right_arm_control:main',
            '05_left_arm_control = rby1_subscriber_pkg.05_left_arm_control:main',
            '06_head_control = rby1_subscriber_pkg.06_head_control:main',
            '07_mobile_control = rby1_subscriber_pkg.07_mobile_control:main',
            '08_DH_dualarm_control_2 = rby1_subscriber_pkg.08_DH_dualarm_control_2:main',
            'controller_manager_ui = rby1_subscriber_pkg.controller_manager_ui:main',
            'sorted_joint_state = rby1_subscriber_pkg.sorted_joint_state:main',           
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)
