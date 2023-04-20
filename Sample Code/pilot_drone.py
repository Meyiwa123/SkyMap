#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleControlMode, PositionTarget


class PilotDrone(Node):

    def __init__(self):
        super().__init__('pilot_drone')

        self.control_mode_publisher = self.create_publisher(OffboardControlMode,
                                                            '/fmu/in/offboard_control_mode',
                                                            10)
        self.position_target_publisher = self.create_publisher(PositionTarget,
                                                               '/fmu/in/position_target',
                                                               10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand,
                                                               '/fmu/in/vehicle_command',
                                                               10)

        self.control_mode = OffboardControlMode()
        self.control_mode.position = True
        self.control_mode.velocity = False
        self.control_mode.acceleration = False
        self.control_mode.attitude = False
        self.control_mode.body_rate = False

        self.vehicle_command = VehicleCommand()
        self.vehicle_command.param1 = 1.0
        self.vehicle_command.param2 = 0.0
        self.vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        self.vehicle_command.target_system = 1
        self.vehicle_command.target_component = 1
        self.vehicle_command.source_system = 1
        self.vehicle_command.source_component = 1
        self.vehicle_command.from_external = True
        self.vehicle_command.timestamp = 0

        self.position_target = PositionTarget()
        self.position_target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        self.position_target.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + \
            PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + \
            PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ
        self.position_target.position.x = 0.0
        self.position_target.position.y = 0.0
        self.position_target.position.z = 0.0

        self.subscription = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, 10)

    def vehicle_control_mode_callback(self, msg):
        if msg.flag_armed and msg.flag_external_manual_override_ok:
            self.control_mode_publisher.publish(self.control_mode)
            self.position_target_publisher.publish(self.position_target)
            self.vehicle_command_publisher.publish(self.vehicle_command)

    def shutdown(self):
        self.control_mode.position = False
        self.control_mode_publisher.publish(self.control_mode)

        self.vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.vehicle_command.param1 = 0.0
        self.vehicle_command_publisher.publish(self.vehicle_command)

        super().shutdown()


def main(args=None):
    rclpy.init(args=args)

    pilot_drone = PilotDrone()

    rclpy.spin(pilot_drone)

    pilot_drone.shutdown()
    pilot_drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
