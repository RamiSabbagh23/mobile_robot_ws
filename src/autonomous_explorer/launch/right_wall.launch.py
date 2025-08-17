from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        Node(
            package='autonomous_explorer',
            executable='right_wall_follower',
            name='right_wall_follower',
            output='screen',
            parameters=[{
                'scan_topic': LaunchConfiguration('scan_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                
                # Motion
                'linear_speed': 0.18,
                'angular_speed': 1.0,
                'max_yaw_rate_scale': 1.0, # <-- Added this parameter

                # Distances (meters)
                'desired_right_dist': 0.55,
                'stop_front': 0.35,
                'grab_right': 0.80,

                # Sector widths (deg) and centers (deg)
                'front_width_deg': 30.0,    # <-- Added this parameter
                'side_width_deg': 40.0,     # <-- Added this parameter
                'diag_width_deg': 30.0,     # <-- Added this parameter
                'front_center_deg': 0.0,    # <-- Added this parameter
                'right_center_deg': -90.0,  # <-- Added this parameter
                'diag_center_deg': -45.0,   # <-- Added this parameter

                # Control
                'k_p': 1.1,
                'bias_corner': 0.4,
            }]
        )
    ])