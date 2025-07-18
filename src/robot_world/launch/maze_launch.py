from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_world_dir = get_package_share_directory('robot_world')

    world_path = os.path.join(robot_world_dir, 'worlds', 'maze.world')
    model_path = os.path.join(robot_world_dir, 'models')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen',
            additional_env={
                'GAZEBO_MODEL_PATH': model_path
            }
        )
    ])
