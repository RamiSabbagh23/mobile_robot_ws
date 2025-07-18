from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths
    robot_world_dir = get_package_share_directory('robot_world')
    robot_description_dir = get_package_share_directory('robot_description')

    # Paths to world and robot
    world_path = os.path.join(robot_world_dir, 'worlds', 'maze.world')
    model_path = os.path.join(robot_description_dir, 'models', 'pioneer2dx', 'model.sdf')
    gazebo_model_path = os.path.join(robot_world_dir, 'models')

    return LaunchDescription([
        # Start Gazebo with maze.world
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
                'GAZEBO_MODEL_PATH': gazebo_model_path
            }
        ),

        # Spawn the pioneer robot from its model.sdf
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

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_link_broadcaster',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        #     output='screen'
        # ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['0.2', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),

        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.2', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        )
    ])