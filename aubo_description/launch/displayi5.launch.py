import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('model')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('aubo_description'))

    print(robot_model)
    xacro_file = os.path.join(pkg_path,'urdf','aubo_i5.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'model',
            default_value='aubo_i3',
            description='Model = (aubo_i3,aubo_i10)'),

        node_robot_state_publisher
    ])