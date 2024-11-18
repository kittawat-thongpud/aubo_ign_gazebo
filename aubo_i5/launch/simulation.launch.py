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
import xacro
import yaml
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
from moveit_configs_utils.launches import generate_demo_launch
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("aubo_i5", package_name="aubo_i5").to_moveit_configs()
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    publish_frequency = LaunchConfiguration('publish_frequency', default="15.0")
    allow_trajectory_execution = LaunchConfiguration('allow_trajectory_execution')
    
    xacro_file = os.path.join(get_package_share_directory('aubo_i5'),
                              'config',
                              'aubo_i5.urdf.xacro')
    
    world_file = os.path.join(get_package_share_directory('aubo_i5'),
                              'config',
                              'world.sdf')

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
            },
        ],
    )
    # Command-line argument: RVIZ file?
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'aubo_i5', '-allow_renaming', 'true'],
    )

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": allow_trajectory_execution,
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": True}, 
    ]

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        )
    """
    parameters=[
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        moveit_simple_controllers_yaml,
        planning_scene_monitor_parameters,
        {"use_sim_time": True}, 
    ],
    )"""
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    aubo_i5_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'aubo_i5_arm_controller'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
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
        bridge,
        # ROS2_CONTROL:
        robot_state_publisher,
        gz_spawn_entity,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments={
            'gz_args': TextSubstitution(
                text=f"-r -v 4 {world_file}"
            )
        }.items(),),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[aubo_i5_arm_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=aubo_i5_arm_controller,
                on_exit=[
                        # MoveIt!2:
                            TimerAction(
                                period=2.0,
                                actions=[
                                    rviz_node_full,
                                    run_move_group_node
                                ]
                            ),
                        ],
            )
        ),
    ])
