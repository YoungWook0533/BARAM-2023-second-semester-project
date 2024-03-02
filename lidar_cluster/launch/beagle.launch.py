import os

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, LogInfo
# from launch.conditions import IfCondition
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_cluster',
            executable='beagle0_core',
            name='beagle0_core',
            output='screen',
        ),
        Node(
            package='lidar_cluster',
            executable='beagle1_core',
            name='beagle1_core',
            output='screen',
        ),
        Node(
            package='lidar_cluster',
            executable='beagle0_moved_path',
            name='beagle0_moved_path',
            output='screen',
        ),
        Node(
            package='lidar_cluster',
            executable='beagle1_moved_path',
            name='beagle1_moved_path',
            output='screen',
        ),
        Node(
            package='lidar_cluster',
            executable='subscribe_map0',
            name='subscribe_map0',
            output='screen',
        ), 
        Node(
            package='lidar_cluster',
            executable='subscribe_map1',
            name='subscribe_map1',
            output='screen',
        ),                        
                
    ])