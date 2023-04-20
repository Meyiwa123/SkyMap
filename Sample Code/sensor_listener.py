#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, QoSInitialization
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix


class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_combined_and_gps_listener')

        sensor_combined_qos_profile = QoSProfile(depth=10, reliability=QoSProfile.ReliabilityPolicy.BEST_EFFORT)
        sensor_combined_qos = QoSInitialization(history=sensor_combined_qos_profile.history, depth=sensor_combined_qos_profile.depth, reliability=sensor_combined_qos_profile.reliability)
        self.sensor_combined_subscription = self.create_subscription(Imu, '/fmu/data/imu', self.sensor_combined_callback, sensor_combined_qos)

        sensor_gps_qos_profile = QoSProfile(depth=10, reliability=QoSProfile.ReliabilityPolicy.BEST_EFFORT)
        sensor_gps_qos = QoSInitialization(history=sensor_gps_qos_profile.history, depth=sensor_gps_qos_profile.depth, reliability=sensor_gps_qos_profile.reliability)
        self.sensor_gps_subscription = self.create_subscription(NavSatFix, '/fmu/data/gps', self.sensor_gps_callback, sensor_gps_qos)

    def sensor_combined_callback(self, msg):
        self.get_logger().info('Received SENSOR COMBINED data: linear acceleration = (%f, %f, %f), angular velocity = (%f, %f, %f), orientation = (%f, %f, %f, %f)' %
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))

    def sensor_gps_callback(self, msg):
        self.get_logger().info('Received SENSOR GPS data: timestamp = %f, latitude = %f, longitude = %f, altitude = %f' %
            (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg.latitude, msg.longitude, msg.altitude))

def main(args=None):
    rclpy.init(args=args)
    sensor_combined_and_gps_listener = SensorListener()
    rclpy.spin(sensor_combined_and_gps_listener)
    sensor_combined_and_gps_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
