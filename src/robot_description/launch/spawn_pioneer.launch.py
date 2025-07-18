from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to Gazebo's classic empty world launch
    gazebo_launch_file = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    # Path to the model.sdf inside installed share directory
    model_path = os.path.join(
        get_package_share_directory('robot_description'),
        'models',
        'pioneer2dx',
        'model.sdf'
    )

    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_pioneer2dx',
            arguments=[
                '-entity', 'pioneer2dx',
                '-file', model_path,
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
    ])
