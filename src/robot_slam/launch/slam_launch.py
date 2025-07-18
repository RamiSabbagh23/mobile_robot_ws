from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_frame': 'base_link'
        }],
        remappings=[('scan', '/scan')]
    )

    return LaunchDescription([
        TimerAction(
            period=5.0,  # השהייה של 5 שניות
            actions=[slam_node]
        )
    ])
