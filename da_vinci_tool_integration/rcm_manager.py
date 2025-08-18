#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener

class RCMManager(Node):
    def __init__(self):
        super().__init__('rcm_manager')

        self.rcm_point = None
        self.rcm_mode = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(Bool, '/set_rcm_trigger', self.set_rcm_callback, 10)

        # Publishers
        self.rcm_mode_pub = self.create_publisher(Bool, '/rcm_mode', 10)
        self.rcm_marker_pub = self.create_publisher(Marker, '/rcm_marker', 10)

        self.get_logger().info("🧠 RCM Manager initialized - will get RCM point from TF")

    def set_rcm_callback(self, msg):
        if msg.data:
            try:
                # Get current tool tip position from TF
                transform = self.tf_buffer.lookup_transform(
                    "lbr_link_0", 
                    "PSM_tool_virtual_tip", 
                    rclpy.time.Time()
                )
                
                # Set RCM point to current tool position
                self.rcm_point = transform.transform
                self.rcm_mode = True
                
                self.get_logger().info(f"📍 RCM automatically set at current tool position: x={self.rcm_point.translation.x:.3f}, y={self.rcm_point.translation.y:.3f}, z={self.rcm_point.translation.z:.3f}")

                # Publish RCM marker
                marker = Marker()
                marker.header.frame_id = 'lbr_link_0'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'rcm_point'
                marker.id = 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = self.rcm_point.translation.x
                marker.pose.position.y = self.rcm_point.translation.y
                marker.pose.position.z = self.rcm_point.translation.z
                marker.pose.orientation.x = self.rcm_point.rotation.x
                marker.pose.orientation.y = self.rcm_point.rotation.y
                marker.pose.orientation.z = self.rcm_point.rotation.z
                marker.pose.orientation.w = self.rcm_point.rotation.w
                marker.scale.x = 0.03
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0

                self.rcm_marker_pub.publish(marker)

                # Publish RCM mode flag
                flag = Bool()
                flag.data = True
                self.rcm_mode_pub.publish(flag)

                self.get_logger().info("✅ RCM mode activated and marker published automatically from TF.")
                
            except Exception as e:
                self.get_logger().error(f"❌ Failed to get RCM point from TF: {e}")
                self.get_logger().error("Make sure the robot is running and TF is available")

def main(args=None):
    rclpy.init(args=args)
    node = RCMManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 