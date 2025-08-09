from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),

    (os.path.join('share', package_name, 'models/pioneer2dx'), glob('models/pioneer2dx/*.*')),
    (os.path.join('share', package_name, 'models/pioneer2dx/meshes'), glob('models/pioneer2dx/meshes/*.*')),

],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot description with Gazebo SDF',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)