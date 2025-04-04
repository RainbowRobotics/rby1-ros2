from setuptools import setup

package_name = 'rby1_mobile_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ian',
    maintainer_email='your_email@example.com',
    description='ROS2 mobile robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobile_publisher = rby1_mobile_control.mobile_publisher:main'
        ],
    },
)
