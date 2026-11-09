#!/usr/bin/env python3
"""Isolated Nav2 Collision Monitor sensor-to-command stop-gate test. No Gazebo/hardware."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]
def generate_launch_description():
    return LaunchDescription([
        Node(package="tf2_ros",executable="static_transform_publisher",name="test_robot_odometry_tf",
             arguments=["--x","0","--y","0","--z","0","--roll","0","--pitch","0","--yaw","0",
                        "--frame-id","odom","--child-frame-id","base_footprint"],output="screen"),
        Node(package="tf2_ros",executable="static_transform_publisher",name="test_scan_static_tf",
             arguments=["--x","0","--y","0","--z","0","--roll","0","--pitch","0","--yaw","0",
                        "--frame-id","base_footprint","--child-frame-id","base_scan"],output="screen"),
        Node(package="nav2_collision_monitor",executable="collision_monitor",name="collision_monitor",
             parameters=[str(ROOT/"config/phase5_collision_monitor_stop.yaml"),{"use_sim_time":False}],
             output="screen"),
        Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_collision_monitor",
             parameters=[{"use_sim_time":False,"autostart":True,"node_names":["collision_monitor"]}],
             output="screen")
    ])
