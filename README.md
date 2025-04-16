# turtle_bot
ROS2-Based Café Butler Robot with NAV2 and GUI Integration
# ROS2-Based Café Butler Robot with NAV2 and GUI Integration

**Author**: Subramaniyan M  
**Department**: Robotics and Automation  
**College**: Sri Ramakrishna Engineering College  
**Specialization**: ROS2, Computer Vision, and Robotic Systems  
**Date**: 2025-04-16  

## Project Overview

This project involves the development of an autonomous Café Butler Robot using ROS2, designed to automate food delivery tasks in a simulated café environment. The robot integrates the NAV2 stack for autonomous navigation and a custom GUI for task management, enabling task-based deliveries in real-time. The system uses Cartographer SLAM for environment mapping and localization via AMCL.

### Key Features:
- Café simulation in Gazebo with tables, kitchen, and docking station
- Cartographer SLAM for environment mapping
- Autonomous navigation with NAV2 stack
- Task management via a custom PyQt GUI
- Real-time obstacle avoidance and path planning
- Validated with 7 real-time test cases

## Requirements
- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS 2 Version**: Humble
- **Simulation Environment**: Gazebo 11

## Tools & Frameworks
- ROS2 Humble Hawksbill
- Navigation2 (NAV2) for autonomous navigation
- Tkinter/PyQt for GUI development
- Python for ROS2 nodes
- Rviz for visualization

## System Architecture & Flow
- **Modular ROS2 Nodes**: Separate nodes for GUI input, navigation, and task handling
- **NAV2 Integration**: Handles path planning, navigation, and goal execution
- **State Machine Logic**: Transition between robot states (Idle → Kitchen → Table → Dock)
- **Task Feedback Loop**: Sends status updates back to the GUI for re-tasking if needed

## Setup Instructions
1. **Set Up ROS2 Environment**:
    ```bash
    $ source /opt/ros/humble/setup.bash
    $ echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

2. **Install Required Packages**:
    ```bash
    $ sudo apt install gazebo11
    $ sudo apt install ros-humble-gazebo-ros-pkgs
    $ sudo apt install ros-humble-cartographer
    $ sudo apt install ros-humble-navigation2
    ```

3. **Create Workspace**:
    ```bash
    $ mkdir -p ~/turtle_bot/src
    $ cd ~/turtle_bot/src
    ```

4. **Clone Dependencies**:
    ```bash
    $ git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    $ git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
    ```

5. **Build Workspace**:
    ```bash
    $ cd ~/turtle_bot
    $ colcon build
    $ source install/setup.bash
    ```

6. **Launch Custom Café Simulation**:
    ```bash
    $ ros2 launch turtlebot3_gazebo cafe_world.launch.py
    ```

## Test Cases
- **Test Case 1**: Launch TurtleBot in Café World
    ```bash
    $ ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```

- **Test Case 2**: Run Cartographer SLAM
    ```bash
    $ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
    ```

- **Test Case 3**: Save the Map
    ```bash
    $ ros2 run nav2_map_server map_saver_cli -f ~/butler_bot/src/maps/cafe_map
    ```

- **Test Case 4**: Launch NAV2 with Saved Map
    ```bash
    $ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=~/butler_bot/src/maps/cafe_map.yaml
    ```

## Contact
- **E-mail**: Subramaniyan.2210051@srec.ac.in
- **Phone**: +91 7708241016
