import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # burger_config = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_amcl_config.yaml')
    # burger_1_config = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_1_amcl_config.yaml')
    # map_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')
    burger_config = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger_amcl_config.yaml'
    burger_1_config = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger_1_amcl_config.yaml'
    map_file = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml'

    rviz_config_dir1 = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/rviz/burger_navigation2.rviz'
    rviz_config_dir2 = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/rviz/burger_1_navigation2.rviz'

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            namespace='burger',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[burger_config]
        ),

        Node(
            namespace='burger_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[burger_1_config]
        ),

        Node(
            namespace='burger',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir1,],
            parameters=[{'use_sim_time': True}],
            output='screen'),

        Node(
            namespace='burger_1',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir2,],
            parameters=[{'use_sim_time': True}],
            output='screen'),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'burger/amcl', 'burger_1/amcl']}]
        )
    ])
