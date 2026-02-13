#!/usr/bin/env python3

"""

Manual Mode Launch File

Starts nodes needed for manual (teleop) control of the robot

Usage:

    ros2 launch launch_pkg manual_mode.launch.py

    

"""

from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    """

    Generates launch description for manual mode.

    Add/uncomment nodes as packages become available.

    """

    

    return LaunchDescription([

        

        # Teleop Node - Keyboard control

        Node(

            package='teleop_pkg',

            executable='teleop_node',

            name='teleop_node',

            output='screen',

            parameters=[{

                'linear_speed': 0.5,

                'angular_speed': 1.0

            }]

        ),

        

        # LiDAR Driver

        # Uncomment when sensing_pkg is ready

        # Node(

        #     package='sensing_pkg',

        #     executable='lidar_driver',

        #     name='lidar_driver',

        #     output='screen'

        # ),

        

        # Base Controller 

        # Uncomment when locomotion_pkg is ready

        # Node(

        #     package='locomotion_pkg',

        #     executable='base_controller',

        #     name='base_controller',

        #     output='screen'

        # ),

    ])
