# Random Point based autonomous exploration

## Overview

This repository contains python code for a random point based autonomous exploration using ROS (Robot Operating System). The purpose of this project is to enable the Turtlebot3 to explore unknown environments autonomously by generating random goals for the robot to reach them.

## Prerequisites

Before using this code, make sure you have the following installed:

- ROS (Robot Operating System)
- TurtleBot Navigation and SLAM packages
- you can find all turtlebot packages, including the navigation and SLAM ones in the robotis github: https://github.com/ROBOTIS-GIT/turtlebot3

## Installation

1. Clone this repository into your ROS workspace:

    ```bash
    git clone https://github.com/JorgeETorradoF/Turtlebot3-Random-Point-Based-Autonomous-Exploration.git
    ```

2. Build the ROS packages:

    ```bash
    cd your_ros_workspace
    catkin_make
    ```

## Usage

1. Connect to the turtlebot3 via ssh (the full guide for connecting to the turtlebot3 can be found in the section 3 Quick start guide of this robotis page: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) 


2. Launch the bring-up in the turtlebot3 console:

    ```bash
    roslaunch turtlebot3_bringup turtlebot3_robot.launch
    ```

3. Launch SLAM in your pc:

    ```bash
    roslaunch turtlebot3_slam turtlebot3_slam.launch
    ```

4. Run the autonomous exploration node:

    ```bash
    rosrun <name of your ros package that contains the autonomous exploration> AutonomousExploration.py
    ```

## Customization

To integrate this code into your specific ROS package, follow these steps:

1. Copy the contents of the `autonomous_exploration` folder into your ROS package.

2. Make necessary modifications in your package's configuration files to include the exploration functionalities.

3. Update your package's dependencies in the `CMakeLists.txt` and `package.xml` files.
