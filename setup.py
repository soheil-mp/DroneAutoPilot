from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_nav_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/models', []),
        ('share/' + package_name + '/meshes', glob('meshes/*.dae')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS2 Drone Navigation System with Reinforcement Learning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_agent = drone_nav_rl.scripts.train_agent:main',
            'evaluate_agent = drone_nav_rl.scripts.evaluate_agent:main',
            'mission_controller = drone_nav_rl.src.utilities.mission_controller:main',
            'path_planner = drone_nav_rl.src.utilities.path_planner:main',
            'mission_visualization = drone_nav_rl.src.utilities.mission_visualization:main',
        ],
    },
) 