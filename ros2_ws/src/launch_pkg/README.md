# Launch Package

Launch files for starting the  robot in different modes.

## Purpose

Makes it easy to start multiple nodes with a single command instead of opening many terminals.

## Launch Files

### manual_mode.launch.py

Starts nodes for manual teleop control.

**Usage:**

```bash

ros2 launch launch_pkg manual_mode.launch.py

```

**Currently starts:**

- teleop_node (keyboard control)

**Will start when ready:**

- lidar_driver (sensing)

- base_controller (motor control)

### test_launch.launch.py

Test launch file using example nodes to verify system works.

**Usage:**

```bash

ros2 launch launch_pkg test_launch.launch.py

```

## Configuration

Edit config/robot_params.yaml to adjust robot parameters.

## Adding New Nodes

To add a node to a launch file:

1. Edit the appropriate .launch.py file

2. Uncomment or add a new Node() entry

3. Rebuild: colcon build --packages-select launch_pkg

## Status

- Basic launch infrastructure

- Manual mode launch file

- Test launch file

- Waiting for other packages to integrate
