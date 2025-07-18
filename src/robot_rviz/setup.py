from setuptools import setup

package_name = 'robot_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/rviz.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/robot_config.rviz']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    description='RViz launch and config for Pioneer2dx',
    license='MIT',
)
