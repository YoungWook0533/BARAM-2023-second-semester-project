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
            executable='lidar_cluster',
            name='lidar_cluster',
            output='screen',
        ),
        Node(
            package='lidar_cluster',
            executable='goal_update',
            name='goal_update',
            output='screen',
        ),        
    ])