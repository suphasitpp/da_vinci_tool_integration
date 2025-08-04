#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class RCMMarkerPublisher(Node):
    def __init__(self):
        super().__init__('rcm_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)  # Publish every 0.5 sec
        
        self.get_logger().info('RCM Marker Publisher started')
        self.get_logger().info('Publishing marker at RCM point: (0.045, 0.940, 0.437)')

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'lbr_link_0'  # base frame or world frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'rcm_marker'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # RCM point coordinates from your log
        marker.pose.position.x = 0.045
        marker.pose.position.y = 0.940
        marker.pose.position.z = 0.437

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05  # 5 cm diameter sphere
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Fully opaque

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RCMMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('RCM Marker Publisher stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 