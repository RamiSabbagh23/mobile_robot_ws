from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rami',
    maintainer_email='ramisabbagh23@gmail.com',
    description='SLAM integration using slam_toolbox',
    license='MIT',  # או כל רישיון אחר שתרצה
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
