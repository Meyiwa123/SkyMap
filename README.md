# SkyMap: A Distributed Consensus-Based Swarm Robotics System for Real-Time 3D Mapping and Object Detection
The goal of this project is to develop a swarm robotics system that can create a 3D map of an area through distributed consensus, using reinforcement learning and computer vision techniques.

## System Overview
The system will consist of multiple drones, with one acting as the “pilot” and the others as “followers”, all communicating through a mesh network topology. The pilot drone will be responsible for exploring the area and collecting data, while the follower drones will assist in mapping the area and providing backup in case of communication or hardware failures.

The drones will be controlled using ROS2, with a control system that utilizes the particle model to maintain formation and avoid collisions. The system will incorporate distributed consensus algorithms to ensure that all drones have a consistent view of the environment, even in the event of communication failures. Machine learning techniques, including reinforcement learning, will be used to optimize the search and automatically identify objects within the search area.

The ground station will be responsible for commanding the drones, simulating the drones, and creating a 3D map from the data. Computer vision techniques will be used to identify objects within the search area, and the resulting map will be displayed in real-time. The system will also be able to simulate the drones’ position in real-time during an actual mission, using Gazebo and RViz.

A web interface will be developed to provide real-time status updates for each drone, including its current position, camera feed, and additional information. The interface will also display the current 3D map and provide tools to control the drone search.

## System Architecture
The system will consist of the following components:

* `Multiple drones`, including a pilot drone and follower drones, all communicating through a mesh network topology.
* `A control system` for each drone, utilizing the ROS2 and px4_ros_com library.
* `A control system for the swarm`, utilizing the particle model to maintain formation and avoid collisions.
* `Distributed consensus algorithms` to ensure all drones have a consistent view of the environment.
* `Machine learning techniques`, including reinforcement learning, to optimize the search and identify objects within the search area.
* `A ground station` to simulate and command the drones.

## Contents of the Repository
This repository contains the following directories:
* `ROS2_control:` Contains the ROS2 packages for controlling the swarm of drones
* `Gazebo_simulation:` Contains the files for simulating the drones' movement in Gazebo
* `Computer_vision:` Contains the code for identifying objects within the search area
* `3D_mapping:` Contains the code for creating a 3D map from the collected data
How to Use
* `Route_planning:` Contains the code for computing optimal routes for the drones.
* `Website:` Contains the code for the website that displays drone status, camera feeds, and 3D map, and allows for drone control.

Please refer to the README files in each directory for instructions on how to use the code.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
