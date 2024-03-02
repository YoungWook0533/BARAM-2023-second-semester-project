# import os

# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node

# import xacro


# def generate_launch_description():

#     name0 = "burger"
#     name1 = "burger_1"

#     # Check if we're told to use sim time
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # Process the URDF file
#     xacro_file = '/home/youngwook/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/robot.urdf.xacro'
#     robot_description_config = xacro.process_file(xacro_file)

#     # Create a robot_state_publisher node
#     params = {'frame_prefix': name0 + '/','robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         namespace=name0,
#         output='screen',
#         parameters=[params]
#     )


#     # Launch!
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='false',
#             description='Use sim time if true'),

#         node_robot_state_publisher
#     ])

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    name0 = "burger"
    name1 = "burger_1"

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    xacro_file = '/home/youngwook/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/urdf/robot.urdf.xacro'
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {
        'frame_prefix': name0 + '/',  # Set the namespace as the frame prefix
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=name0,  # Set the namespace for the node
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
