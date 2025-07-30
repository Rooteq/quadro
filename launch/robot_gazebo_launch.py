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

    xacro_file = os.path.join(pkg_path, "description", "robot.xacro")
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

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_path, "description","meshes")
        + ":"
        + os.path.join(pkg_path, "description")
        + ":"
        + os.path.join(pkg_path, "description", "world"),
    )

    arguments = LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value="world", description="Gz sim World"
            ),
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        # launch_arguments=[("gz_args", [LaunchConfiguration("world"), ".sdf", " -r"])],
        # launch_arguments=[("gz_args", [LaunchConfiguration("world"), ".sdf", " -r -v 4" " --physics-engine gz-physics-bullet-featherstone-plugin"])],
        launch_arguments=[("gz_args", ["empty.sdf", " -r" " --physics-engine gz-physics-bullet-featherstone-plugin"])],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",robot_desc,
            "-x","0.5",
            "-y","0.5",
            "-z","0.07",
            "-R","0.0",
            "-P","0.0",
            "-Y","0.0",
            "-name","robot",
            "-allow_renaming","true",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_path, "config", "bridge_params.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
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

    delayed_spawn = TimerAction(period=5.0, actions=[gz_spawn_entity])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'urdf_file',
                default_value=xacro_file,
                description='Path to URDF file'
            ),
            
            DeclareLaunchArgument(
                'end_effector_frame',
                default_value='link',
                description='Name of end-effector frame'
            ),

            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_diff_drive_controller],
                )
            ),
            gazebo_resource_path,
            arguments,
            gazebo,
            node_robot_state_publisher,
            delayed_spawn,
            bridge,
            rviz,

            # Node(
            #     package='quadro',
            #     executable='torque_controller',
            #     name='forward_kinematics_node',
            #     arguments=[
            #         LaunchConfiguration('urdf_file'),
            #         LaunchConfiguration('end_effector_frame')
            #     ],
            #     output='screen'
            # )

        ]
    )