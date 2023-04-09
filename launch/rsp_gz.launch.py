import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

# from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_name = "bob"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(pkg_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            ]
        ),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "bob"],
        output="screen",
    )

    joint_state_broadcaster_spawn = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )

    delay_JSB_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[joint_state_broadcaster_spawn],
        )
    )

    robot_controllers = ["diff_drive_base_controller"]
    robot_controller_spawners = []

    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    delay_controller_after_JSB = []

    for controller in robot_controller_spawners:
        delay_controller_after_JSB += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawn,
                    on_exit=[TimerAction(period=3.0, actions=[controller])],
                )
            )
        ]

    return LaunchDescription(
        [rsp, gazebo, spawn_entity, delay_JSB_after_gazebo] + delay_controller_after_JSB
    )
