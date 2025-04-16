# Real-Time Cartesian Control of UR Robot via MoveIt Servo

This repository provides the necessary steps to achieve **real-time Cartesian position control** of a UR robot using **MoveIt Servo**.

> **Please use:** Ubuntu 24.04 and ROS 2 Jazzy


## Prerequisites

Before proceeding, ensure you have the [hardware](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html#robot-setup) setup and the [network](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/network_setup.html#network-setup) configured properly so you can communicate with the robot. You can test this by pinging the robot.

## Setup Instructions

1. **Install ROS 2 Jazzy**  
   Follow the official installation guide:  
   [ROS 2 Jazzy Installation (Ubuntu)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

2. **Install the UR Robot Driver**  
   Install the package with:
   ```bash
   sudo apt-get install ros-jazzy-ur
   ```
   [UR Robot Driver Repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

3. **Set up UR robot by generating kinematics param file**
   Create a folder somewhere on your PC to store this file. Generate the file as [shown here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/installation/robot_setup.html#extract-calibration-information)

4. **Install ROS 2 Control package**
    Follow the official installation guide:  
    [ros_2_control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)

   ## Testing Instructions

   Now you should be able to run this command:
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur16e robot_ip:=<robot_ip> kinematics_params_file:= <path_to_yaml_calibration_file> headless_mode:=True
   ```
   and a rviz window should open and you should be able to see a virtual simulated robot.
   Next, run this commmand 
   
