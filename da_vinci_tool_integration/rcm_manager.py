#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class RCMManager(Node):
    def __init__(self):
        super().__init__('rcm_manager')

        self.rcm_point = None
        self.rcm_mode = False
        self.latest_candidate_pose = None

        # Subscriptions
        self.create_subscription(PoseStamped, '/rcm_candidate_pose', self.rcm_pose_callback, 10)
        self.create_subscription(Bool, '/set_rcm_trigger', self.set_rcm_callback, 10)

        # Publishers
        self.rcm_mode_pub = self.create_publisher(Bool, '/rcm_mode', 10)
        self.rcm_marker_pub = self.create_publisher(Marker, '/rcm_marker', 10)

        self.get_logger().info("🧠 RCM Manager initialized")

    def rcm_pose_callback(self, msg):
        self.latest_candidate_pose = msg

    def set_rcm_callback(self, msg):
        if msg.data and self.latest_candidate_pose:
            self.rcm_point = self.latest_candidate_pose.pose
            self.rcm_mode = True
            self.get_logger().info(f"📍 RCM set at: x={self.rcm_point.position.x:.3f}, y={self.rcm_point.position.y:.3f}, z={self.rcm_point.position.z:.3f}")

            # Publish RCM marker
            marker = Marker()
            marker.header.frame_id = 'lbr_link_0'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'rcm_point'
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = self.rcm_point
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

            self.get_logger().info("✅ RCM mode activated and marker published.")

def main(args=None):
    rclpy.init(args=args)
    node = RCMManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 