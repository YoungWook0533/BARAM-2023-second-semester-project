# import os

# from ament_index_python.packages import get_package_share_directory


# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition
# from launch_ros.actions import Node



# def generate_launch_description():

#     package_name='turtlebot3_gazebo'
#     name0 = "burger"
#     name1 = "burger_1"

#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#     world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'worlds','empty_world.world')    

#     # ros2 launch my_robot rsp.launch.py
#     rsp = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','rsp.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'false'}.items()
#     )

#     #ros2 launch gazebo_ros gazebo.launch.py
#     gazebo = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
#              )

#     # ros2 run gazebo_ros spawn entity.py -topic robot_description -entity bot_name
#     spawn_entity1 = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-topic', 'burger/robot_description',
#             '-entity', name0,
#             '-x', '0.000000',
#             '-y', '0.000000',
#             '-z', '0.000000',
#             '-robot_namespace', name0,  # Set the robot namespace here
#         ],
#         output='screen'
#     )


#     # rviz = Node(
#     #     package='rviz2',
#     #     executable='rviz2',
#     #     arguments=['-d', os.path.join(get_package_share_directory(package_name), 'config', 'maze.rviz')],
#     #     # condition=IfCondition(LaunchConfiguration('rviz'))
#     # )

#     # Launch them all!s
#     return LaunchDescription([
#         rsp,
#         gazebo,
#         spawn_entity1,
#         #rviz
#     ])

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    package_name = 'turtlebot3_gazebo'
    name0 = "burger"
    name1 = "burger_1"

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', 'empty_world.world')

    # ros2 launch gazebo_ros gazebo.launch.py
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # ros2 run gazebo_ros spawn entity.py -topic robot_description -entity bot_name
    spawn_entity1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'burger/robot_description',
            '-entity', name0,
            '-x', '0.000000',
            '-y', '0.000000',
            '-z', '0.000000',
            '-robot_namespace', name0,  # Set the robot namespace here
        ],
        output='screen'
    )

    # Process the URDF file
    xacro_file = '/home/youngwook/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/robot.urdf.xacro'
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {
        'frame_prefix': name0 + '/',  # Set the namespace as the frame prefix
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name0,  # Set the namespace for the node
        output='screen',
        parameters=[params]
    )

    # Launch them all!
    return LaunchDescription([
        # gazebo,
        # spawn_entity1,
        robot_state_publisher1
    ])

