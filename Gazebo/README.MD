# Gazbo Manual

This guide provides the full setup for the Gazebo sim using **ROS 2 Humble** and **Ignition Gazebo**.

---

## SECTION 1:  FILES
* **robot.urdf**: Physical robot anatomy & Lidar sensor.
* **newcastle_world.sdf**: Simulation environment & physics.

---

## SECTION 2: EXECUTION (4 TERMINALS)
```bash
Terminal 1: Start Gazebo
ign gazebo newcastle_world.sdf

Terminal 2: Spawn Robot
ros2 run ros_gz_sim create -file robot.urdf -name my_box_bot

Terminal 3: Start Bridge (Gazebo Topics > ROS2 Topics)
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /joint_states@sensor_msgs/msg/JointState@gz.msgs.Model \
  /model/my_box_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args -r /model/my_box_bot/tf:=/tf -p use_sim_time:=true

Terminal 4: Keyboard Control
ros2 run teleop_pkg teleop_node 
```

## SECTION 3: RVIZ (SEE WHAT ROBOT SEES)
```bash
1.Start RVIZ in terminal 1: 
rviz2

2.Tell ros2 how the robot parts are connected in terminal 2: 
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/Documents/NCL_Lunabotics_Challenge-main/Gazebo/robot.urdf)" -p use_sim_time:=true

3. Fixed Frame: 
Change map to base_link.

4. Add topic: 
Click "Add" -> "LaserScan".

5. Select Topic: 
Set to /scan.
```

## SECTION 4: SLAM (BUILD A MAP)
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```
