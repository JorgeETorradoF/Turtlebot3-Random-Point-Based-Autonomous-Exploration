# Random Point based autonomous exploration

## Overview

This repository contains code for autonomous exploration based on random points using ROS (Robot Operating System). The purpose of this project is to enable robots to explore unknown environments efficiently by generating random points for exploration.

## Prerequisites

Before using this code, make sure you have the following installed:

- ROS (Robot Operating System)
- TurtleBot Navigation and SLAM packages
- you can find all turtlebot packages, including the navigation and SLAM ones here: https://github.com/ROBOTIS-GIT/turtlebot3

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

1. Launch the TurtleBot:

    ```bash
    roslaunch turtlebot_bringup minimal.launch
    ```

2. Launch the SLAM node:

    ```bash
    roslaunch turtlebot_navigation gmapping_demo.launch
    ```

3. Launch the autonomous exploration node:

    ```bash
    roslaunch autonomous_exploration explore.launch
    ```

## Customization

To integrate this code into your specific ROS package, follow these steps:

1. Copy the contents of the `autonomous_exploration` folder into your ROS package.

2. Make necessary modifications in your package's configuration files to include the exploration functionalities.

3. Update your package's dependencies in the `CMakeLists.txt` and `package.xml` files.

## Contributing

Feel free to contribute to this project by opening issues or pull requests. Your feedback and improvements are highly appreciated!

## License

This project is licensed under the [MIT License](LICENSE).
