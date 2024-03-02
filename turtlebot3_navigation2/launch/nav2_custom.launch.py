import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = True

    controller_yaml_burger = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_controller.yaml')
    bt_navigator_yaml_burger = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_bt_navigator.yaml')
    planner_yaml_burger = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_planner_server.yaml')
    behavior_yaml_burger = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_behavior.yaml')
    burger_config = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger_amcl_config.yaml'
    rviz_config_dir1 = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/rviz/burger_navigation2.rviz'

    controller_yaml_burger_1 = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_1_controller.yaml')
    bt_navigator_yaml_burger_1 = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_1_bt_navigator.yaml')
    planner_yaml_burger_1 = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_1_planner_server.yaml')
    behavior_yaml_burger_1 = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'param', 'burger_1_behavior.yaml')
    burger_1_config = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/param/burger_1_amcl_config.yaml'
    rviz_config_dir2 = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/rviz/burger_1_navigation2.rviz'

    map_file = '/home/youngwook/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation2/map/map.yaml'    
    
    return LaunchDescription([     

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]),

        # Nodes for burger

        Node(
            namespace='burger',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[burger_config]),

        Node(
            namespace='burger',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir1,],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),                                
    
        Node(
            namespace='burger',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_burger]),

        Node(
            namespace='burger',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_burger]),
            
        Node(
            namespace='burger',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_yaml_burger],
            output='screen'),

        Node(
            namespace='burger',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_burger]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='burger_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                                        'burger/planner_server',
                                        'burger/controller_server',
                                        'burger/behavior_server',
                                        'burger/bt_navigator'
                                        ]}]),
        
        # Nodes for burger_1
        
        Node(
            namespace='burger_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[burger_1_config]),

        Node(
            namespace='burger_1',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir2,],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),        

        Node(
            namespace='burger_1',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_burger_1]),

        Node(
            namespace='burger_1',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_burger_1]),
            
        Node(
            namespace='burger_1',
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_yaml_burger_1],
            output='screen'),

        Node(
            namespace='burger_1',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_burger_1]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='burger_1_lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                                        'burger_1/planner_server',
                                        'burger_1/controller_server',
                                        'burger_1/behavior_server',
                                        'burger_1/bt_navigator'
                                        ]}]),

        # Node for lifecycle_manager     
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'burger/amcl', 'burger_1/amcl']}]
        )

    ])
