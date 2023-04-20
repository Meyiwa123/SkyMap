#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode
from geometry_msgs.msg import PoseStamped, Vector3, Point, TwistStamped

import math
import numpy as np

'''
# FormationMsg.msg

int32 drone_id
int32 num_followers
float32[3] com_position
float32[3] com_velocity
'''

class FormationController(Node):
    def __init(self, drone_id, num_followers):
        super().__init__('formation_controller')
        self.drone_id = drone_id
        self.num_followers = num_followers
        self.k = 0.3    # Gain for the formation controller

        self.target_position = Point()
        self.target_velocity = Vector3()
        self.current_position = Point()
        self.current_velocity = Vector3()

        self.subscriber = self.create_subscription(
            FormationMsg, '/formation_info', self.formation_callback, 10)
        self.publisher = self.create_publisher(
            PoseStamped, 'offboard_pose', 10)
        
    def formation_callback(self, msg):
        # Only calculate formation for this drone's formation message
        if msg.drone_id != self.drone_id:
            return

        # Calculate position of center of mass
        com_position = Point()
        for i in range(msg.num_followers + 1):
            if i == self.drone_id:
                com_position.x += self.current_position.x
                com_position.y += self.current_position.y
                com_position.z += self.current_position.z
            else:
                com_position.x += msg.com_position[i].x
                com_position.y += msg.com_position[i].y
                com_position.z += msg.com_position[i].z
        com_position.x /= (msg.num_followers + 1)
        com_position.y /= (msg.num_followers + 1)
        com_position.z /= (msg.num_followers + 1)

        # Calculate velocity of center of mass
        com_velocity = Vector3()
        for i in range(msg.num_followers + 1):
            if i == self.drone_id:
                com_velocity.x += self.current_velocity.x
                com_velocity.y += self.current_velocity.y
                com_velocity.z += self.current_velocity.z
            else:
                com_velocity.x += msg.com_velocity[i].x
                com_velocity.y += msg.com_velocity[i].y
                com_velocity.z += msg.com_velocity[i].z
        com_velocity.x /= (msg.num_followers + 1)
        com_velocity.y /= (msg.num_followers + 1)
        com_velocity.z /= (msg.num_followers + 1)

        # Update target position and velocity using particle model
        if self.num_followers == 0:
            self.target_position.x = com_position.x
            self.target_position.y = com_position.y
            self.target_position.z = com_position.z
        else:
            # Get all follower positions and velocities from the custom topic
            follower_positions = []
            follower_velocities = []
            for i in range(self.num_followers):
                msg = self.follower_msgs[i]
                follower_positions.append(
                    Vector3(x=msg.position.x, y=msg.position.y, z=msg.position.z))
                follower_velocities.append(
                    Vector3(x=msg.velocity.x, y=msg.velocity.y, z=msg.velocity.z))

            # Calculate total mass of the formation
            total_mass = self.drone_mass * (self.num_followers + 1)

            # Calculate center of mass position and velocity
            com_position = Vector3(x=0, y=0, z=0)
            com_velocity = Vector3(x=0, y=0, z=0)
            for i in range(self.num_followers):
                com_position.x += follower_positions[i].x
                com_position.y += follower_positions[i].y
                com_position.z += follower_positions[i].z
                com_velocity.x += follower_velocities[i].x
                com_velocity.y += follower_velocities[i].y
                com_velocity.z += follower_velocities[i].z
            com_position.x += self.current_position.x
            com_position.y += self.current_position.y
            com_position.z += self.current_position.z
            com_velocity.x += self.current_velocity.x
            com_velocity.y += self.current_velocity.y
            com_velocity.z += self.current_velocity.z
            com_position.x /= (self.num_followers + 1)
            com_position.y /= (self.num_followers + 1)
            com_position.z /= (self.num_followers + 1)
            com_velocity.x /= (self.num_followers + 1)
            com_velocity.y /= (self.num_followers + 1)
            com_velocity.z /= (self.num_followers + 1)

            # Calculate self force and velocity
            my_position = Vector3(x=self.current_position.x,
                                y=self.current_position.y, z=self.current_position.z)
            my_velocity = Vector3(x=self.current_velocity.x,
                                y=self.current_velocity.y, z=self.current_velocity.z)
            force = Vector3(x=0, y=0, z=0)
            for i in range(self.num_followers):
                follower_position = follower_positions[i]
                follower_velocity = follower_velocities[i]
                r = (follower_position - my_position).length()
                direction = (follower_position - my_position) / r
                force += direction * self.force_strength / r**2
            my_force = force + (self.drone_mass * com_velocity -
                                total_mass * my_velocity) / self.tau

            self.target_position = com_position
            self.target_velocity.x = com_velocity.x - self.K * \
                (my_velocity.x - my_force.x / self.drone_mass)
            self.target_velocity.y = com_velocity.y - self.K * \
                (my_velocity.y - my_force.y / self.drone_mass)
            self.target_velocity.z = com_velocity.z - self.K * \
                (my_velocity.z - my_force.z / self.drone_mass)


        
