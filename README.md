## Overview

This repository is the integrated version of Path Planning and Obstacle Avoidance Logic(2 seperate roles combined). The integrated

code combines path planning using A* and A* replanning as well as obstacle avoidance logic combined together allowing for path planning and dynamic obstacle

replanning to occur when running the file. The file was in Python which was used for Gazebo and RViz simulations, however final deployment used C++ to be compatible with CPU Usage on the NVIDIA Jetson AGX Orin.

The file also consists of subscribed nodes to LiDAR's object detection node, Camera's stop sign node, GNSS node proving longitude and latitude, as well UI node from the UI Flask providing the destination point. This file also publishes the updated distance node to the UI Flask and the status of the navigated route.

## Tools

Software: C++, Ubuntu, ROS 2 Humble, Bash, CMake
Hardware: NVIDIA Jetson AGX Orin
Simulation Tools: Gazebo, RViz

