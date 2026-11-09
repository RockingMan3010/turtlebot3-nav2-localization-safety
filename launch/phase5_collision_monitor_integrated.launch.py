#!/usr/bin/env python3
"""Task 2 condition A: baseline AMCL global localization + Gazebo odom TF.
This is a NEW controlled-trial launch; it does not change the installed baseline.
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
    robot_sdf = str(ROOT / "models/waffle_gt_A.sdf")
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        include("gazebo_ros", "gzserver.launch.py", world=world),
        include("gazebo_ros", "gzclient.launch.py", condition=IfCondition(gui)),
        include("turtlebot3_gazebo", "robot_state_publisher.launch.py",
                use_sim_time="true"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-entity", "waffle", "-file", robot_sdf,
                        "-x", "-2.0", "-y", "-0.5", "-z", "0.01"],
             output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ROOT / "launch/phase5_bringup_with_safety.launch.py")),
            launch_arguments={"use_sim_time": "true", "map": map_yaml,
                              "params_file": str(ROOT / "config/waffle_experiment_A.yaml")}.items()),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen",
             parameters=[str(ROOT / "config/phase5_collision_monitor_stop.yaml")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_collision_monitor", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["collision_monitor"]}]),
    ])
