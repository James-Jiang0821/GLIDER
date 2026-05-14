from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'glider_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Jiang, Hussain Khan',
    maintainer_email='jamesjiang0821@gmail.com, hussainniassuh786@gmail.com',
    description='ROS 2 nodes for the underwater glider (drivers, control, safety, manager).',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Drivers
            'imu_minimu_node = glider_ros.driver.imu_minimu_node:main',
            'gnss_maxm10s_i2c_node = glider_ros.driver.gnss_maxm10s_i2c_node:main',
            'communication_iridium_node = glider_ros.driver.communication_iridium_node:main',
            'sonar_ping_node = glider_ros.driver.sonar_ping_node:main',
            'pressure_keller_node = glider_ros.driver.pressure_keller_node:main',
            # Bridge
            'can_bridge_node = glider_ros.bridge.can_bridge_node:main',
            # Adapters
            'imu_adapter_node = glider_ros.adapter.imu_adapter_node:main',
            'sonar_adapter_node = glider_ros.adapter.sonar_adapter_node:main',
            'telemetry_adapter_node = glider_ros.adapter.telemetry_adapter_node:main',
            'dead_reckoning_node = glider_ros.adapter.dead_reckoning_node:main',
            # Manager
            'state_manager_node = glider_ros.manager.state_manager_node:main',
            # Safety
            'safety_node = glider_ros.safety.safety_node:main',
            # Controller
            'controller_node = glider_ros.controller.controller_node:main',
            # Mission
            'mission_node = glider_ros.mission.mission_node:main',
        ],
    },
)