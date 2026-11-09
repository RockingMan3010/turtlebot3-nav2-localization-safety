#!/usr/bin/env python3
"""Task 2 condition B: AMCL map localization + EKF-owned odom TF.
Launch only after shutting down the baseline simulation.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]

def include(package, file, condition=None, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / "launch" / file)),
        launch_arguments=kwargs.items(), condition=condition)

def generate_launch_description():
    gui = LaunchConfiguration("gazebo_gui")
    world = str(Path(get_package_share_directory("turtlebot3_gazebo"))
                / "worlds/turtlebot3_world.world")
    map_yaml = str(Path(get_package_share_directory("nav2_bringup"))
                   / "maps/turtlebot3_world.yaml")
    gazebo_gui = include("gazebo_ros", "gzclient.launch.py", condition=IfCondition(gui))
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="false",
                              description="Start Gazebo GUI (headless default saves RAM)"),
        include("gazebo_ros", "gzserver.launch.py", world=world),
        gazebo_gui,
        include("turtlebot3_gazebo", "robot_state_publisher.launch.py",
                use_sim_time="true"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-entity", "waffle", "-file",
                        str(ROOT / "models/waffle_gt_B.sdf"),
                        "-x", "-2.0", "-y", "-0.5", "-z", "0.01"],
             output="screen"),
        Node(package="robot_localization", executable="ekf_node",
             name="ekf_assisted",
             parameters=[str(ROOT / "config/ekf_assisted.yaml")],
             output="screen"),
        include("turtlebot3_navigation2", "navigation2.launch.py",
                use_sim_time="True", map=map_yaml,
                params_file=str(ROOT / "config/waffle_experiment_B.yaml")),
    ])
