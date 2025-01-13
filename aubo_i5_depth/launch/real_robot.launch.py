# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler,TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder, load_yaml, load_xacro
from moveit_configs_utils.launches import generate_demo_launch
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from xacro import process_file

def generate_launch_description():
    package_name = "aubo_i5_depth"
    package_path = Path(get_package_share_directory(package_name))
    moveit_config = (
                    MoveItConfigsBuilder("aubo_i5_depth", package_name=package_name,robot_description="robot_description")
                    .robot_description(file_path="config/aubo_i5_depth_camera.urdf.xacro", mappings={"use_sim_model":"false",})
                    .robot_description_semantic(file_path="config/aubo_i5_depth_camera.srdf")
                    .robot_description_kinematics(file_path="config/kinematics.yaml")
                    .joint_limits(file_path="config/joint_limits.yaml")
                    .trajectory_execution(file_path="config/moveit_controllers.yaml",
                                          moveit_manage_controllers = True)
                    .planning_scene_monitor(
                                            publish_planning_scene = True,
                                            publish_geometry_updates = True,
                                            publish_state_updates = True,
                                            publish_transforms_updates = True,
                                            publish_robot_description = True,
                                            publish_robot_description_semantic = True,
                                            )
                    .sensors_3d(file_path="config/sensors_3d.yaml")
                    .planning_pipelines(default_planning_pipeline = "ompl",
                                        pipelines =['ompl','pilz_industrial_motion_planner','chomp'],
                                        )
                    .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
                    .to_moveit_configs()
                    )
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    robot_ip = LaunchConfiguration('robot_ip', default="192.168.1.202")
    publish_frequency = LaunchConfiguration('publish_frequency', default="15.0")

    joint_names_yaml = load_yaml(package_path/'config'/'joint_names.yaml')
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": str(package_path/'data/ros2.sqlite'),
    }
    # ***** STATIC TRANSFORM ***** #
    # Publish TF:
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": publish_frequency,
                "use_sim_time": use_sim_time
            },
        ],
    )
    # Command-line argument: RVIZ file?
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        warehouse_ros_config,
        {"use_sim_time": use_sim_time}, 
    ]
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    move_group_params = [
        moveit_config.to_dict(),
        warehouse_ros_config,
        {"use_sim_time": use_sim_time}, 
    ]

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        )
    #aubo ros2 driver
    aubo_driver_node = Node(
        package="aubo_controller",
        executable="aubo_controller",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
    )
    #aubo ros2 tarjectory action
    aubo_trajectory_action_node = Node(
        package="aubo_ros2_trajectory_action",
        executable="aubo_ros2_trajectory_action",
        output="screen",
        parameters=[joint_names_yaml],
        arguments=[package_name], 
    )
    '''
    try:
        with open("/home/kittawat/ros2_ws/src/aubo_ign_gazebo/aubo_i5_depth/save.json", 'w') as file:
            json.dump(moveit_config.to_dict(), file, indent=4)  # Save the dictionary as a formatted JSON
    except Exception as e:
        print(f"An error occurred while saving the file: {e}")
    '''
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'robot_ip',
            default_value=robot_ip,
            description='ipv4 address of aubo controller'),
        DeclareBooleanLaunchArg(
            "allow_trajectory_execution", 
            default_value=True),
        DeclareBooleanLaunchArg(
            "publish_monitored_planning_scene", 
            default_value=True),
        DeclareBooleanLaunchArg(
            "monitor_dynamics", 
            default_value=False),
        DeclareLaunchArgument(
            "publish_frequency", 
            default_value="15.0"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),),
        DeclareBooleanLaunchArg(
            "debug", 
            default_value=False),
        # ROS2_CONTROL:
        aubo_driver_node,
        run_move_group_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=run_move_group_node,
                on_exit=[aubo_trajectory_action_node,
                         robot_state_publisher,
                         rviz_node_full],
            )
        ),

    ])

        
