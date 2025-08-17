from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='autonomous_explorer',
            executable='explorer_node',
            name='autonomous_explorer',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Motion & safety params
                'linear_speed': 0.18,
                'angular_speed': 0.7,
                'obstacle_stop_range': 0.45,
                'wall_follow_range': 0.7,
                'min_clear_sector_width_deg': 25.0,
                'backoff_time_s': 1.0,
                'spin_search_time_s': 8.0
            }]
        )
    ])
