# Teleop Package

Keyboard teleoperation node for manual control of the NCL Lunabotics robot.

## Features

- Simple keyboard control (W/A/S/D)

- Publishes to /cmd_vel topic

- Adjustable speed settings

- Safe stop on exit

## Usage

```bash

ros2 run teleop_pkg teleop_node

```

## Controls

- **W** - Move forward

- **S** - Move backward

- **A** - Turn left

- **D** - Turn right

- **Space** - Stop

- **Q** - Quit

## Configuration

Edit teleop_node.py to adjust speeds:

- linear_speed - Forward/backward speed (m/s)

- angular_speed - Turning speed (rad/s)

## Message Type

Publishes geometry_msgs/Twist messages to /cmd_vel:

- linear.x - Forward/backward velocity

- angular.z - Rotational velocity

## Status

✅ Basic keyboard control implemented


