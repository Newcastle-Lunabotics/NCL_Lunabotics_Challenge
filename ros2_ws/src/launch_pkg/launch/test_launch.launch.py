#!/usr/bin/env python3
"""
Test Launch File
Uses example nodes to verify launch system works

Usage:
    ros2 launch launch_pkg test_launch.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        # Start example talker
        Node(
            package='ncl_example_pubsub',
            executable='talker',
            name='test_talker',
            output='screen'
        ),
        
        # Start example listener
        Node(
            package='ncl_example_pubsub',
            executable='listener',
            name='test_listener',
            output='screen'
        ),
    ])
