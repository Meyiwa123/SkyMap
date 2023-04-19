#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode


class PilotDroneControl(Node):
    def __init__(self):
        super().__init__('pilot_drone_control')
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                      "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.target_position = None

    def timer_callback(self):
        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    def takeoff(self, target_position):
        # Set target position
        self.target_position = target_position

        # Change to offboard mode
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        # Arm the drone
        self.arm()

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        # component which should execute the command, 0 for all components
        msg.target_component = 1
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds /
                            1000)  # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds /
                            1000)  # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_
        msg.position = self.target_position
        msg.timestamp = int(Clock().now().nanoseconds /
                            1000)  # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pilot_drone_control = PilotDroneControl()

    target_position = [0, 0, 0]
    pilot_drone_control.takeoff(target_position)
    rclpy.spin(pilot_drone_control)

    pilot_drone_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
