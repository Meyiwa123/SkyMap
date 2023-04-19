#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry


class DroneController(Node):
    def __init__(self, drone_id):
        super().__init__('drone_controller')

        # Initialize drone and relay ID
        self.drone_id = drone_id

        # Subscribe to the scan and vehicle odometry topic
        self.create_subscription(LaserScan, 'lidar', self.lidar_callback, 10)
        self.create_subscription(
            VehicleCommand, 'vehicle_command', self.vehicle_command_callback, 10)
        self.create_subscription(
            VehicleOdometry, 'vehicle_odometry', self.odometry_callback, 10)
        self.create_subscription(
            VehicleStatus, 'vehicle_status', self.vehicle_status_callback, 10)

        # Create publisher to publish vehicle commands
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 'vehicle_command', 10)

        # Create a timer to call timer_callback()
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.log_info('Current mode: %s' % self.current_mode)
        self.log_info('Battery voltage: %s' % self.battery_voltage)

    def vehicle_status_callback(self, msg):
        self.current_mode = msg.vehicle_state
        self.battery_voltage = msg.battery_voltage_v

    def lidar_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.ranges)
        self.send_to_relay('lidar_data', msg)

    def odometry_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        self.send_to_relay('odometry_data', msg)

    def vehicle_command_callback(self, msg):
        # Process message received from ground station
        self.get_logger().info('I heard: "%s"' % msg)

    def send_to_relay(self, topic_name, data):
        # Create and publish message to relay topic
        relay_topic_name = '/relay/{}/{}'.format(self.drone_id, topic_name)
        relay_msg = String()
        relay_msg.data = str(data)

        self.create_publisher(String, relay_topic_name, 10).publish(relay_msg)

    def receive_from_relay(self, msg):
        # Process message received from relay
        topic_name = msg.topic
        if topic_name == '/relay/{}/vehicle_command'.format(self.drone_id):
                # Parse the message as a VehicleCommand message
                vehicle_command = VehicleCommand()
                vehicle_command.deserialize(msg.data.encode('utf-8'))

                # Publish the received VehicleCommand to the vehicle_command topic
                self.get_logger().info('Received vehicle command: %s' % str(vehicle_command))
                self.vehicle_command_publisher.publish(vehicle_command)

        for i in range(1, 11):
            if topic_name.startswith('/relay/{}/'.format(i)):
                self.get_logger().info('Received relay message for drone %d on topic %s' % (i, msg.data))
                break

        else:
            self.get_logger().warning('Received message on unknown topic: %s' % topic_name)
        


def main(args=None):
    rclpy.init(args=args)

    drone_controller = DroneController(1)

    # Create subscriber to relay topic
    relay_topic_name = '/relay/{}/#'.format(drone_controller.drone_id)
    drone_controller.create_subscription(
        String, relay_topic_name, drone_controller.receive_from_relay, 10)

    rclpy.spin(drone_controller)

    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
