#!/usr/bin/env python3
"""Start only the GUI when myco_sdk_node is already running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="MyCo-5-0.80"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.56.103"),
        Node(
            package="myco_basic_api",
            executable="myco_gui_new.py",
            name="myco_gui",
            output="screen",
            parameters=[{
                "robot_model": LaunchConfiguration("robot_model"),
                "robot_ip": LaunchConfiguration("robot_ip"),
            }],
        ),
    ])
