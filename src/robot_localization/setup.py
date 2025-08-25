from glob import glob
from setuptools import setup

package_name = 'robot_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rami',
    maintainer_email='your@email.com',
    description='Manual localization with map server and AMCL (no Nav2)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_visualizer_node = robot_localization.map_visualizer_node:main',
        ],
    },
)