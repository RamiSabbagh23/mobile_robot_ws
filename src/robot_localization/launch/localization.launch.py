#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path
import os
import xacro


def _resolve_paths(context):
    """
    Resolve paths and build the node list.
    Default map: src/autonomous_explorer/map/my_map1.yaml
    """

    # --- Packages we rely on ---
    pkg_autonomous = 'autonomous_explorer'   # map lives here
    pkg_robot_local = 'robot_localization'   # your package
    pkg_robot_desc  = 'robot_description'    # URDF/Xacro
    pkg_nav2_amcl   = 'nav2_amcl'            # AMCL executable

    # --- Share directories ---
    share_robot_local = Path(get_package_share_directory(pkg_robot_local))
    share_robot_desc  = Path(get_package_share_directory(pkg_robot_desc))

    # Config defaults (only for AMCL + RViz)
    default_amcl_yaml = share_robot_local / 'config' / 'amcl.yaml'
    default_rviz_cfg  = share_robot_local / 'config' / 'rviz_config.rviz'
    urdf_xacro        = share_robot_desc / 'urdf' / 'pioneer2dx.urdf.xacro'

    # Map handling
    map_yaml_arg = (LaunchConfiguration('map_yaml_path').perform(context) or '').strip()
    if map_yaml_arg:
        # User provided a path
        map_yaml_path = os.path.abspath(os.path.expanduser(os.path.expandvars(map_yaml_arg)))
    else:
        # Default to src/autonomous_explorer/map/my_map1.yaml
        ws_root = Path(os.getcwd()).resolve()
        candidate = ws_root / 'src' / 'autonomous_explorer' / 'map' / 'my_map1.yaml'
        map_yaml_path = str(candidate)

    # Other configs
    amcl_config = Path(LaunchConfiguration('amcl_config').perform(context) or default_amcl_yaml)
    rviz_config = Path(LaunchConfiguration('rviz_config').perform(context) or default_rviz_cfg)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('1', 'true', 'yes')

    # Resolve Xacro -> robot_description
    robot_description_xml = xacro.process_file(str(urdf_xacro)).toxml()
    robot_description = ParameterValue(robot_description_xml, value_type=str)

    # --- Nodes ---
    nodes = [

        # 1) Map visualizer publishes /map (TRANSIENT_LOCAL)
        Node(
            package=pkg_robot_local,
            executable='map_visualizer_node',
            name='map_visualizer_node',
            output='screen',
            parameters=[{
                'map_yaml_path': map_yaml_path
            }]
        ),

        # 2) Delay then start AMCL
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package=pkg_nav2_amcl,
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[str(amcl_config)],
                )
            ]
        ),

        # 3) Delay then configure AMCL lifecycle
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
                    output='screen'
                )
            ]
        ),

        # 4) Delay then activate AMCL
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
                    output='screen'
                )
            ]
        ),

        # 5) Delay then start robot_state_publisher + RViz
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'robot_description': robot_description
                    }]
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', str(rviz_config)]
                )
            ]
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_path',
            default_value='',
            description='Path to map YAML (default: src/autonomous_explorer/map/my_map1.yaml)'
        ),
        DeclareLaunchArgument(
            'amcl_config',
            default_value='',
            description='Path to AMCL YAML (default: <robot_localization>/config/amcl.yaml)'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='Path to RViz config (default: <robot_localization>/config/rviz_config.rviz)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated time'
        ),
        OpaqueFunction(function=_resolve_paths),
    ])
