from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('robot_slam'),
        'config',
        'slam_config.yaml'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_path],
        remappings=[('scan', '/scan')]
    )

    return LaunchDescription([
        TimerAction(
            period=0.0,  # Pause Time
            actions=[slam_node]
        )
    ])
