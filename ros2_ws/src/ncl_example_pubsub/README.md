NCL Example ROS 2 Pub/Sub (Humble)

This is a minimal working example ROS 2 package for the NCL Lunabotics team.

It shows:

how to create a ROS 2 Python package

a talker node that publishes messages

a listener node that subscribes and prints messages

how to build + run it in a workspace

This package is meant as the template for new team members.

What this package contains

Package name: ncl_example_pubsub
Nodes:

talker → publishes std_msgs/String on /chatter

listener → subscribes to /chatter and prints received messages

Files to look at:

ncl_example_pubsub/talker.py

ncl_example_pubsub/listener.py

setup.py (registers nodes as executables)

Requirements

Ubuntu  (Ubuntu 22.04)

ROS 2 Humble installed

Python 3

Check ROS is sourced:

source /opt/ros/humble/setup.bash

Repo structure reminder

This repository contains a ROS workspace inside:

ros2_ws/
  src/
    ncl_example_pubsub/
      ncl_example_pubsub/
      package.xml
      setup.py

Step 1 — clone the repo
cd ~
git clone https://github.com/SahasT23/NCL_Lunabotics_Challenge.git
cd NCL_Lunabotics_Challenge

Step 2 — build the workspace

Always build from the workspace root (ros2_ws):

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build


If you only want to build this one package:

colcon build --packages-select ncl_example_pubsub

Step 3 — source the workspace overlay

This makes ROS aware of the newly built package:

source install/setup.bash


Tip: you must do this in every new terminal (unless you add it to ~/.bashrc).

Step 4 — run the nodes (2 terminals)
Terminal A (listener)
cd ~/NCL_Lunabotics_Challenge/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ncl_example_pubsub listener

Terminal B (talker)
cd ~/NCL_Lunabotics_Challenge/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ncl_example_pubsub talker


You should see the listener printing messages.

Useful debug commands

List active topics:

ros2 topic list


Watch the messages:

ros2 topic echo /chatter


Node list:

ros2 node list


Info about a node:

ros2 node info /talker

How to create a new package (team template)

Create packages inside:
ros2_ws/src/

Example:

cd ~/NCL_Lunabotics_Challenge/ros2_ws/src
ros2 pkg create <your_package_name> --build-type ament_python --dependencies rclpy std_msgs


Then:

add your node python file(s) into <your_package_name>/<your_package_name>/

register them in setup.py under entry_points

build + source + run
