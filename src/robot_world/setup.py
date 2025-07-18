from setuptools import setup
import os
from glob import glob

package_name = 'robot_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        # Install world file
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # Install model files
        (os.path.join('share', package_name, 'models', 'maze_world'), glob('models/maze_world/*.*')),
        (os.path.join('share', package_name, 'models', 'maze_world', 'meshes'), glob('models/maze_world/meshes/*.*')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Custom maze world for Gazebo',
    license='MIT',
)
