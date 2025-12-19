import os
import xacro

# TODO: fix use sim time as a settable parameter

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("quadro"))

    # Check if we're told to use sim time
    sim_mode = LaunchConfiguration("sim_mode", default=True)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    xacro_file = os.path.join(pkg_path, "description", "leg.xacro")
    robot_desc = Command(["xacro ", xacro_file, " sim_mode:=", sim_mode])
    robot_state_publisher_params = {
        "robot_description": robot_desc,
        "use_sim_time": use_sim_time,
    }

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_state_publisher_params],
    )
    
    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "forward_position_controller",
        ],
        output="screen",
    )
    rviz_config_file = os.path.join(pkg_path, "config", "rviz_config.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    my_controller = Node(
        package='quadro',
        executable='torque_controller',
        name='my_controller',
        arguments=[robot_desc, 'link'],
        output='screen'
    )


    return LaunchDescription(
        [
            node_robot_state_publisher,
            rviz,
        ]
    )