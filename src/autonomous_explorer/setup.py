from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_explorer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Simple reactive explorer that drives the robot to map with SLAM.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = autonomous_explorer.explorer_node:main',
            'astar_plan = autonomous_explorer.astar_plan:main',
            'pure_pursuit_follow = autonomous_explorer.pure_pursuit_follow:main',
        ],
    },
)
