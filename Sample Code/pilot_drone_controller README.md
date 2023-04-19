# Drone Controller
This Python code is a ROS2 node that controls a drone. It subscribes to topics that provide lidar data, vehicle odometry data, vehicle status data and vehicle commands, and publishes vehicle commands.

## Installation
To use this code, you need to have ROS2 installed on your machine. You also need to have the following ROS2 packages installed:

* std_msgs
* sensor_msgs
* px4_msgs

## Usage
The DroneController class subscribes to the following topics:

* `/lidar:` for lidar data
* `/vehicle_command:` for vehicle commands
* `/vehicle_odometry:` for vehicle odometry data
* `/vehicle_status:` for vehicle status data
* It also publishes to the `/vehicle_command topic`.

The `timer_callback` function is called at a frequency of 5 seconds and displays the current mode and battery voltage of the drone.

The `lidar_callback`, `odometry_callback` and `vehicle_command_callback` functions are called when data is received from the respective subscribed topics. The `vehicle_status_callback` function is called when vehicle status data is received.

The `send_to_relay` function is used to publish data to a relay topic with a specific drone ID. The `receive_from_relay` function is called when data is received from a relay topic.
