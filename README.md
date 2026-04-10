## Overview

This repository is the integrated version of Path Planning and Obstacle Avoidance Logic(2 seperate roles combined). The integrated code combines path planning using A* and A* replanning as well as obstacle avoidance logic combined together allowing for path planning and dynamic obstacle replanning to occur when running the file. The file was in Python which was used for Gazebo and RViz simulations, however final deployment used C++ to be compatible with CPU Usage on the NVIDIA Jetson AGX Orin.The file also consists of subscribed nodes to LiDAR's object detection node, Camera's stop sign node, GNSS node proving longitude and latitude, as well UI node from the UI Flask providing the destination point. This file also publishes the updated distance node to the UI Flask and the status of the navigated route.

## Tools

Software: C++, Ubuntu 22.04, ROS 2 Humble, Bash, CMake

Hardware: NVIDIA Jetson AGX Orin

Simulation Tools: Gazebo, RViz

## System Flow

UI Goal -> GPS Conversion -> A* Planning -> Obstacle Avoidance Logic -> A* Replanning -> Output

## Topics(Subscribed)
User Interface Destination(UI Flask.py)
Topic: /ui/destination_point

Type: geometry_msgs/msg/Point

Description: Destination Point (lat, lon)

LiDAR Node

Topic: /velodyne_points

Type: sensor_msgs/msg/PointCloud2

Description: Raw 3D point cloud for obstacle detection

Stop Sign Detection (from Camera)

Topic: /aav/stop_sign_detected

Type: std_msgs/msg/Bool

Description: Stop sign presence indicator

GNSS Node(GPS)

Topic: /aav/gps/fix

Type: sensor_msgs/msg/NavSatFix

Description: Current Location 

## How to Build

Prerequisites

```bash
sudo apt install \
ros-humble-desktop \  
ros-humble-pcl-ros \    
libpcl-dev ```

Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src```

**Make sure to copy navigation_system.cpp to navigation_cpp/src/

'''cd ~/ros2_ws

colcon build --packages-select navigation_cpp

source install/setup.bash'''

## How to Run
Mock data using mock coordinates for the /ui/destination_point using ros 2 terminals

Terminal 1:

```bash
ros2 run navigation_cpp path_planning_module```

Terminal 2:

```bash
ros2 topic pub --once /ui/destination_point geometry_msgs/msg/Point "{x: 45.3849, y: -75.6959, z: 0.0}"; sleep 2; 
ros2 topic pub --once /ui/destination_point geometry_msgs/msg/Point "{x: 45.3850, y: -75.6957, z: 0.0}"; sleep 2; 
ros2 topic pub --once /ui/destination_point geometry_msgs/msg/Point "{x: 45.3851, y: -75.6955, z: 0.0}" ```

## Expected Output

Terminal 1:
 [INFO] [1775792057.210518948] [path_planning_module]: Goal sent from UI Team
[INFO] [1775792057.211507597] [path_planning_module]:    GPS Info:  (45.384900°, -75.695900°)
[INFO] [1775792057.211670463] [path_planning_module]:  Status: planning
[INFO] [1775792057.213357166] [path_planning_module]:  A*: 47 waypoints in 1 ms
[INFO] [1775792057.213482989] [path_planning_module]: D*Lite initialized: start(75,75) goal(121,119)
[INFO] [1775792057.213640607] [path_planning_module]:  Status: navigating
[INFO] [1775792057.213716526] [path_planning_module]:    Data being sent to UI Flask:
[INFO] [1775792057.213729267] [path_planning_module]:    Distance:  32.1 meters
[INFO] [1775792057.213741799] [path_planning_module]:    Status:    navigating
[INFO] [1775792059.263741966] [path_planning_module]: 
[INFO] [1775792059.263904844] [path_planning_module]:    Listening on: /ui/destination_point
[INFO] [1775792059.263918597] [path_planning_module]:    Publishing to: /nav/path_distance, /nav/nav_status
[INFO] [1775792059.263927002] [path_planning_module]: 
[INFO] [1775792060.701894085] [path_planning_module]: Goal sent from UI Team
[INFO] [1775792060.702024228] [path_planning_module]:    GPS Info:  (45.385000°, -75.695700°)
[INFO] [1775792060.702079072] [path_planning_module]:  Status: planning
[INFO] [1775792060.702106689] [path_planning_module]:  Status: replanning
[INFO] [1775792060.751358838] [path_planning_module]:  D*Lite: 1 waypoints in 48 ms
[INFO] [1775792060.751793232] [path_planning_module]:  Status: navigating
[INFO] [1775792060.751816953] [path_planning_module]:    Data being sent to UI Flask:
[INFO] [1775792060.751826178] [path_planning_module]:    Distance:  0.0 meters
[INFO] [1775792060.751838109] [path_planning_module]:    Status:    navigating
[INFO] [1775792064.005423063] [path_planning_module]: Goal sent from UI Team
[INFO] [1775792064.005530899] [path_planning_module]:    GPS Info:  (45.385100°, -75.695500°)
[INFO] [1775792064.005559389] [path_planning_module]:  Status: planning
[INFO] [1775792064.005570849] [path_planning_module]:  Status: replanning
[INFO] [1775792064.039019570] [path_planning_module]:  D*Lite: 1 waypoints in 33 ms
[INFO] [1775792064.039447070] [path_planning_module]:  Status: navigating
[INFO] [1775792064.039555037] [path_planning_module]:    Data being sent to UI Flask:
[INFO] [1775792064.039578037] [path_planning_module]:    Distance:  0.0 meters
[INFO] [1775792064.039603791] [path_planning_module]:    Status: navigating


## Next Steps
1.) Test with Cameras, GNSS for a vehicle test to ensure real world data can interact with the navigation module
2.) Test with real data with the UI Destination point using the NVIDIA JETSON AGX Orin 
3.) Validate results and ensure for correct functionality when finishing the integration and full system integration process

