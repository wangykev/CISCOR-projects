# treadmill_node.py (ROS 2 node to control WEG CFW-11 via Modbus)

from setuptools import setup
from setuptools import find_packages

package_name = 'cfw11_ros2_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['cfw11_ros2_control', 'cfw11_ros2_control.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pymodbus'],
    zip_safe=True,
    maintainer='ciscor',
    maintainer_email='wangykev@gmail.com',
    description='ROS 2 Modbus control for WEG CFW-11',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'treadmill_node = cfw11_ros2_control.treadmill_node:main',
        ],
    },
)
