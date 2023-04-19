#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.msg import OccupancyGrid, MapMetaData
from nav2_msgs.srv import SaveMap

from px4_msgs.msg import VehicleOdometry


class MapBuilder(Node):
    def __init__(self):
        super().__init__('map_builder')

        self.map_resolution = 0.1  # meters per pixel
        self.scan_threshold = 0.2  # minimum distance for an obstacle in meters
        self.current_map = None
        self.current_pose = None
        self.current_map_2d = None
        self.current_pose = None
        self.map_meta_data = None

        self.create_subscription(
            VehicleOdometry, 'vehicle_odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)

    def odom_callback(self, msg):
        self.current_pose = msg.pose

    def scan_callback(self, msg):
        if self.current_pose is None:
            return

        # Convert laser scan data to a 2D occupancy grid
        map_width = int(msg.range_max / self.map_resolution)
        map_height = int((msg.angle_max - msg.angle_min) /
                         msg.angle_increment / self.map_resolution)
        map_data = [-1] * (map_width * map_height)

        for i, scan_range in enumerate(msg.ranges):
            if scan_range <= msg.range_min or scan_range >= msg.range_max:
                continue

            scan_angle = msg.angle_min + i * msg.angle_increment
            x = self.current_pose.position.x + \
                scan_range * math.cos(scan_angle)
            y = self.current_pose.position.y + \
                scan_range * math.sin(scan_angle)

            map_x = int(x / self.map_resolution)
            map_y = int(y / self.map_resolution)

            if map_x < 0 or map_x >= map_width or map_y < 0 or map_y >= map_height:
                continue

            index = map_y * map_width + map_x
            if map_data[index] < 0 or scan_range < map_data[index]:
                map_data[index] = int(scan_range / self.scan_threshold * 100)

        # Publish the 2D map
        if self.current_map_2d is None:
            self.current_map_2d = OccupancyGrid()
            self.current_map_2d.header.frame_id = 'map'
            self.map_meta_data = MapMetaData()
            self.map_meta_data.resolution = self.map_resolution
            self.map_meta_data.width = map_width
            self.map_meta_data.height = map_height
            self.map_meta_data.origin.position.x = self.current_pose.position.x - \
                map_width * self.map_resolution / 2
            self.map_meta_data.origin.position.y = self.current_pose.position.y - \
                map_height * self.map_resolution / 2
            self.map_meta_data.origin.orientation.w = 1.0
            self.current_map_2d.info = self.map_meta_data

        self.current_map.header.stamp = self.get_clock().now().to_msg()
        self.current_map.data = map_data
        self.map_pub.publish(self.current_map)

        # Create the 3D map
        if self.current_map_3d is None:
            self.current_map_3d = OccupancyGrid()
            self.current_map_3d.header.frame_id = 'map'
            self.current_map_3d.info.resolution = self.map_resolution
            self.current_map_3d.info.width = map_width
            self.current_map_3d.info.height = map_height
            self.current_map_3d.info.origin.position.x = self.current_pose.position.x - \
                map_width * self.map_resolution / 2
            self.current_map_3d.info.origin.position.y = self.current_pose.position.y - \
                map_height * self.map_resolution / 2
            self.current_map_3d.info.origin.position.z = 0
            self.current_map_3d.info.origin.orientation.x = 0
            self.current_map_3d.info.origin.orientation.y = 0
            self.current_map_3d.info.origin.orientation.z = 0
            self.current_map_3d.info.origin.orientation.w = 1.0

        # Convert the 2D map to a 3D map
        map_data_3d = [-1] * (map_width * map_height * self.map_height)
        for i in range(map_width):
            for j in range(map_height):
                for k in range(self.map_height):
                    index_2d = j * map_width + i
                    index_3d = k * map_width * map_height + index_2d
                    if map_data[index_2d] >= 0 and k < map_data[index_2d]:
                        map_data_3d[index_3d] = 100
                    elif k < self.map_height / 2:
                        map_data_3d[index_3d] = 0

        # Publish the 3D map
        self.current_map_3d.header.stamp = self.get_clock().now().to_msg()
        self.current_map_3d.data = map_data_3d
        self.map_3d_pub.publish(self.current_map_3d)


def main(args=None):
    rclpy.init(args=args)
    map_builder = MapBuilder()
    rclpy.spin(map_builder)
    map_builder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
