#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Quaternion, Vector3

class StaticSurgicalBox(Node):
    def __init__(self):
        super().__init__('static_surgical_box')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info("📦 Static surgical box marker running")

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        
        # Box position and orientation
        box_x = 0.7
        box_y = 0.2
        box_z = 0.4  # center of 20cm tall box
        
        # Publish main surgical box
        self._publish_box_marker(now, box_x, box_y, box_z)
        
        # Publish corner markers
        self._publish_corner_markers(now, box_x, box_y, box_z)
        
        self.get_logger().info("📦 Published surgical box and corner markers")

    def _publish_box_marker(self, now, x, y, z):
        """Publish the main surgical box marker"""
        box = Marker()
        box.header.frame_id = 'world'
        box.header.stamp = now
        box.ns = 'surgical_box'
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = x
        box.pose.position.y = y
        box.pose.position.z = z
        box.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)  # 90 degrees around Z
        box.scale = Vector3(x=0.4, y=0.2, z=0.2)  # 40cm x 20cm x 20cm rectangle
        box.color = ColorRGBA(r=0.5, g=0.4, b=0.3, a=0.3)  # semi-transparent
        box.lifetime.sec = 0
        self.marker_pub.publish(box)

    def _publish_corner_markers(self, now, box_x, box_y, box_z):
        """Publish 4 green corner dots around the surgical box"""
        # Box dimensions and offset
        box_width = 0.4   # 40cm
        box_depth = 0.2   # 20cm
        box_height = 0.2  # 20cm
        corner_offset = 0.02  # 2cm inside from edges
        corner_z = box_z - box_height/2 + corner_offset  # bottom of box + 2cm higher
        
        # Corner positions (after 90° rotation)
        corners = [
            # Front-left
            (box_x - box_depth/2 + corner_offset, box_y - box_width/2 + corner_offset),
            # Front-right  
            (box_x + box_depth/2 - corner_offset, box_y - box_width/2 + corner_offset),
            # Back-left
            (box_x - box_depth/2 + corner_offset, box_y + box_width/2 - corner_offset),
            # Back-right
            (box_x + box_depth/2 - corner_offset, box_y + box_width/2 - corner_offset)
        ]
        
        # Publish each corner marker
        for i, (corner_x, corner_y) in enumerate(corners):
            corner = Marker()
            corner.header.frame_id = 'world'
            corner.header.stamp = now
            corner.ns = 'surgical_box'
            corner.id = 2 + i  # Start from ID 2
            corner.type = Marker.SPHERE
            corner.action = Marker.ADD
            corner.pose.position.x = corner_x
            corner.pose.position.y = corner_y
            corner.pose.position.z = corner_z
            corner.pose.orientation = Quaternion(w=1.0)
            corner.scale = Vector3(x=0.01, y=0.01, z=0.01)  # 1cm sphere
            corner.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # solid green
            corner.lifetime.sec = 0
            self.marker_pub.publish(corner)

def main(args=None):
    rclpy.init(args=args)
    node = StaticSurgicalBox()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 