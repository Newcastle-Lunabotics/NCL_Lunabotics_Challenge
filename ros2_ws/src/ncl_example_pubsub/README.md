NCL ROS 2 Example: Publisher / Subscriber (Humble)

This package is a minimal, fully working ROS 2 example for the
Newcastle University Lunabotics Challenge robot.

It is intended as:

a learning reference for new team members

a template for creating future ROS 2 packages

a sanity-check that your ROS 2 workspace is set up correctly

What this package demonstrates

This package shows:

how a ROS 2 Python package is structured

how to write a publisher node

how to write a subscriber node

how nodes communicate using topics

how to build and run nodes inside a workspace

Package overview

Package name: ncl_example_pubsub

Nodes included:

Node	Purpose
talker	Publishes text messages on a topic
listener	Subscribes to the topic and prints messages

Topic used:

/chatter   (type: std_msgs/String)

Folder structure
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

Requirements

Ubuntu (typically 22.04 for Humble)

ROS 2 Humble

Python 3

Git

Check ROS is installed and sourced:

source /opt/ros/humble/setup.bash
echo $ROS_DISTRO


You should see:

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

This makes ROS aware of newly built packages:

source install/setup.bash


You must do this every time you open a new terminal.

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


You should see messages printed in the listener terminal.

How ROS 2 communication works (simple explanation)

Talker node

creates a publisher

sends messages on /chatter

Listener node

subscribes to /chatter

receives messages

prints them to the terminal

They do not talk directly — ROS handles message passing.

Useful debugging commands

List all active nodes:

ros2 node list


List all topics:

ros2 topic list


Watch messages on a topic:

ros2 topic echo /chatter


Get info about a node:

ros2 node info /listener

How to create your own package (team template)

From the workspace src directory:

cd ros2_ws/src
ros2 pkg create my_package_name \
  --build-type ament_python \
  --dependencies rclpy std_msgs


Then:

Put your node code in my_package_name/my_package_name/

Register the node in setup.py

Build + source + run
