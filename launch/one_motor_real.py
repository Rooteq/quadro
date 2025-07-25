#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_path = get_package_share_directory('quadro')
    default_rviz_config = os.path.join(pkg_path, 'config', 'rviz_real.rviz')
    robot_controllers = os.path.join(pkg_path, "config", "one_motor_params.yaml")    

    xacro_file = os.path.join(pkg_path,'description','one_motor.xacro')
    robot_desc = Command(['xacro ', xacro_file])


    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description' : robot_description},
                    robot_controllers],
        # arguments=["--ros-args", "--log-level", "DEBUG"],
        emulate_tty=True,
        output="screen"
    )

    joint_traj_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        emulate_tty=True,
        output="screen"
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        emulate_tty=True,
        output="screen"
    )

    delayed_joint_traj_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_traj_spawner]
            )
    )

    delayed_broad_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_broad_spawner]
            )
    )

    delayed_controller_manager= TimerAction(period=2.0, actions=[controller_manager])


    return LaunchDescription([
        # Launch arguments
        # model_path_arg,
        # model_file_arg,
        # use_rviz_arg,
        # Nodes
        delayed_controller_manager,
        delayed_broad_spawner,
        delayed_joint_traj_spawner,
        robot_state_publisher_node,
        rviz_node
    ])