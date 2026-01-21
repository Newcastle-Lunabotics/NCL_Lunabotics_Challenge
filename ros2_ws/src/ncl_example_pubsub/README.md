# NCL ROS 2 Example: Publisher / Subscriber (Humble)

This package is a minimal, fully working ROS 2 example for the
Newcastle University Lunabotics team.

It serves as:

a learning reference for new team members

a template for future ROS 2 packages

a quick way to verify your ROS 2 setup works

## Package Overview

Package name: ncl_example_pubsub

Included nodes
talker	Publishes messages periodically (published /chatter topic)

listener	Subscribes and prints messages
Topic used
/chatter   (std_msgs/String)

## Folder Structure
ncl_example_pubsub/
├── ncl_example_pubsub/
│   ├── __init__.py
│   ├── talker.py        # Publisher node
│   └── listener.py      # Subscriber node
├── resource/
│   └── ncl_example_pubsub
├── test/
├── package.xml
├── setup.py
└── setup.cfg

## Requirements

Ubuntu (22.04)

ROS 2 Humble

Python 3

Git

Check ROS is installed and sourced:

source /opt/ros/humble/setup.bash
echo $ROS_DISTRO


Expected output:

humble

Step 1 — Clone the repository
cd ~
git clone https://github.com/SahasT23/NCL_Lunabotics_Challenge.git
cd NCL_Lunabotics_Challenge

Step 2 — Build the workspace

All ROS 2 packages live inside ros2_ws/src.

Always build from the workspace root:

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build


To build only this package:

colcon build --packages-select ncl_example_pubsub

Step 3 — Source the workspace overlay

You must do this before running nodes:

source install/setup.bash


Do this every time you open a new terminal.

Step 4 — Run the example (two terminals)
Terminal A — Listener
cd ~/NCL_Lunabotics_Challenge/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ncl_example_pubsub listener

Terminal B — Talker
cd ~/NCL_Lunabotics_Challenge/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ncl_example_pubsub talker


You should see messages appearing in the listener terminal.

## How ROS 2 communication works (simple)

Talker node

creates a publisher

sends messages on /chatter

Listener node

subscribes to /chatter

receives messages

prints them to the terminal

Nodes do not talk directly — ROS handles all messaging.

## Useful Debug Commands

List running nodes:

ros2 node list


List active topics:

ros2 topic list


View messages on a topic:

ros2 topic echo /chatter


Inspect a node:

ros2 node info /listener

## Creating Your Own ROS 2 Package (Template)

From the workspace src directory:

cd ros2_ws/src
ros2 pkg create my_package_name \
  --build-type ament_python \
  --dependencies rclpy std_msgs


Then:

Add node code in my_package_name/my_package_name/

Register nodes in setup.py

Build → source → run
