from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'pioneer2dx.urdf.xacro'
    )

    # Process xacro -> URDF string (no shell call, no token issues)
    robot_description_xml = xacro.process_file(urdf_path).toxml()
    robot_description = ParameterValue(robot_description_xml, value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,           # set True only when Gazebo/clock is running
            'robot_description': robot_description
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([rsp, rviz])
