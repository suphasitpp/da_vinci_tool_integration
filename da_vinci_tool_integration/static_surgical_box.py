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
        self.get_logger().info("📦 Static surgical box marker with entry hole running")

    def publish_markers(self):
        now = self.get_clock().now().to_msg()

        # Transparent Box Marker
        box = Marker()
        box.header.frame_id = 'world'
        box.header.stamp = now
        box.ns = 'surgical_box'
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = -0.2
        box.pose.position.y = 0.85
        box.pose.position.z = 0.4  # center of 20cm tall box
        box.pose.orientation = Quaternion(w=1.0)
        box.scale = Vector3(x=0.4, y=0.2, z=0.2)  # 40cm x 20cm x 20cm rectangle
        box.color = ColorRGBA(r=0.5, g=0.4, b=0.3, a=0.3)  # semi-transparent
        box.lifetime.sec = 0
        self.marker_pub.publish(box)
        self.get_logger().info("📦 Published box marker")

        # Red Hole Marker (top center) - COMMENTED OUT
        # hole = Marker()
        # hole.header.frame_id = 'world'
        # hole.header.stamp = now
        # hole.ns = 'surgical_box'
        # hole.id = 1
        # hole.type = Marker.CYLINDER
        # hole.action = Marker.ADD
        # hole.pose.position.x = -0.2
        # hole.pose.position.y = 0.85
        # hole.pose.position.z = 0.51  # top of cube (0.4 + 0.2/2 + small offset)
        # hole.pose.orientation = Quaternion(w=1.0)
        # hole.scale = Vector3(x=0.02, y=0.02, z=0.005)  # 2cm radius, thin height
        # hole.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # solid red
        # hole.lifetime.sec = 0
        # self.marker_pub.publish(hole)

        # Green corner dots at lower corners of the box (2cm inside from surface)
        # Box center is at (-0.2, 0.85, 0.4) with size (0.4, 0.2, 0.2)
        # Lower corners are at z = 0.4 - 0.2/2 = 0.3
        # Moving 2cm (0.02m) inside from edges
        
        # Corner 1: Front-left
        corner1 = Marker()
        corner1.header.frame_id = 'world'
        corner1.header.stamp = now
        corner1.ns = 'surgical_box'
        corner1.id = 2
        corner1.type = Marker.SPHERE
        corner1.action = Marker.ADD
        corner1.pose.position.x = -0.2 - 0.4/2 + 0.02  # -0.38 (left edge + 2cm inside)
        corner1.pose.position.y = 0.85 - 0.2/2 + 0.02  # 0.77 (front edge + 2cm inside)
        corner1.pose.position.z = 0.32  # bottom of box + 2cm higher
        corner1.pose.orientation = Quaternion(w=1.0)
        corner1.scale = Vector3(x=0.01, y=0.01, z=0.01)  # 1cm sphere
        corner1.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # solid green
        corner1.lifetime.sec = 0
        self.marker_pub.publish(corner1)

        # Corner 2: Front-right
        corner2 = Marker()
        corner2.header.frame_id = 'world'
        corner2.header.stamp = now
        corner2.ns = 'surgical_box'
        corner2.id = 3
        corner2.type = Marker.SPHERE
        corner2.action = Marker.ADD
        corner2.pose.position.x = -0.2 + 0.4/2 - 0.02  # -0.02 (right edge - 2cm inside)
        corner2.pose.position.y = 0.85 - 0.2/2 + 0.02  # 0.77 (front edge + 2cm inside)
        corner2.pose.position.z = 0.32  # bottom of box + 2cm higher
        corner2.pose.orientation = Quaternion(w=1.0)
        corner2.scale = Vector3(x=0.01, y=0.01, z=0.01)  # 1cm sphere
        corner2.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # solid green
        corner2.lifetime.sec = 0
        self.marker_pub.publish(corner2)

        # Corner 3: Back-left
        corner3 = Marker()
        corner3.header.frame_id = 'world'
        corner3.header.stamp = now
        corner3.ns = 'surgical_box'
        corner3.id = 4
        corner3.type = Marker.SPHERE
        corner3.action = Marker.ADD
        corner3.pose.position.x = -0.2 - 0.4/2 + 0.02  # -0.38 (left edge + 2cm inside)
        corner3.pose.position.y = 0.85 + 0.2/2 - 0.02  # 0.93 (back edge - 2cm inside)
        corner3.pose.position.z = 0.32  # bottom of box + 2cm higher
        corner3.pose.orientation = Quaternion(w=1.0)
        corner3.scale = Vector3(x=0.01, y=0.01, z=0.01)  # 1cm sphere
        corner3.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # solid green
        corner3.lifetime.sec = 0
        self.marker_pub.publish(corner3)

        # Corner 4: Back-right
        corner4 = Marker()
        corner4.header.frame_id = 'world'
        corner4.header.stamp = now
        corner4.ns = 'surgical_box'
        corner4.id = 5
        corner4.type = Marker.SPHERE
        corner4.action = Marker.ADD
        corner4.pose.position.x = -0.2 + 0.4/2 - 0.02  # -0.02 (right edge - 2cm inside)
        corner4.pose.position.y = 0.85 + 0.2/2 - 0.02  # 0.93 (back edge - 2cm inside)
        corner4.pose.position.z = 0.32  # bottom of box + 2cm higher
        corner4.pose.orientation = Quaternion(w=1.0)
        corner4.scale = Vector3(x=0.01, y=0.01, z=0.01)  # 1cm sphere
        corner4.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # solid green
        corner4.lifetime.sec = 0
        self.marker_pub.publish(corner4)
        self.get_logger().info("🟢 Published 4 green corner dots")

        # 5 Yellow dots in the middle of the box
        center_x = -0.2  # box center x
        center_y = 0.85  # box center y
        center_z = 0.4   # box center z
        radius = 0.03    # 3cm radius circle (smaller than before)
        num_dots = 5     # Only 5 dots
        
        for i in range(num_dots):
            angle = 2 * math.pi * i / num_dots
            dot_x = center_x + radius * math.cos(angle)
            dot_y = center_y + radius * math.sin(angle)
            
            yellow_dot = Marker()
            yellow_dot.header.frame_id = 'world'
            yellow_dot.header.stamp = now
            yellow_dot.ns = 'surgical_box'
            yellow_dot.id = 6 + i  # Start from ID 6
            yellow_dot.type = Marker.SPHERE
            yellow_dot.action = Marker.ADD
            yellow_dot.pose.position.x = dot_x
            yellow_dot.pose.position.y = dot_y
            yellow_dot.pose.position.z = center_z
            yellow_dot.pose.orientation = Quaternion(w=1.0)
            yellow_dot.scale = Vector3(x=0.01, y=0.01, z=0.01)  # 1cm sphere
            yellow_dot.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # solid yellow
            yellow_dot.lifetime.sec = 0
            self.marker_pub.publish(yellow_dot)
        self.get_logger().info("🟡 Published 5 yellow dots")

def main(args=None):
    rclpy.init(args=args)
    node = StaticSurgicalBox()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 