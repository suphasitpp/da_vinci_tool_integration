#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
import time


class PS5TeleopRCM(Node):
    def __init__(self):
        super().__init__('ps5_teleop_rcm')

        # Parameters
        self.deadzone = 0.1       # Stick deadzone
        self.scale_xy = 0.005     # m per step (X/Y)
        self.scale_z = 0.005      # m per step (Z)
        self.publish_rate = 20.0  # Hz

        # Input state
        self.last_msg_time = self.get_clock().now()
        self.last_vector = Vector3()

        # ROS setup
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_nudge = self.create_publisher(Vector3, '/tool_nudge', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.send_nudge)

        self.get_logger().info("🎮 PS5 Teleop RCM Node ready")

    def joy_callback(self, msg):
        # PS5 mapping (corrected):
        # axes[1] = Left stick X (left/right)
        # axes[0] = Left stick Y (up/down, inverted)
        # axes[2] = L2 (1.0 unpressed → -1.0 fully pressed)
        # axes[5] = R2 (1.0 unpressed → -1.0 fully pressed)

        lx = -msg.axes[1]
        ly = -msg.axes[0]
        l2 = msg.axes[2]  # L2 is axes[2]
        r2 = msg.axes[5]  # R2 is axes[5]

        dx = 0.0
        dy = 0.0
        dz = 0.0

        # Apply deadzone
        if abs(lx) > self.deadzone:
            dx = lx * self.scale_xy
        if abs(ly) > self.deadzone:
            dy = -ly * self.scale_xy  # invert Y for natural feel

        # Convert trigger range [1.0 → -1.0] to [0.0 → 1.0]
        dz_in = (1.0 - r2) / 2.0
        dz_out = (1.0 - l2) / 2.0
        dz = (dz_in - dz_out) * self.scale_z

        # Store for sending
        self.last_vector = Vector3(x=dx, y=dy, z=dz)

    def send_nudge(self):
        if abs(self.last_vector.x) > 1e-6 or abs(self.last_vector.y) > 1e-6 or abs(self.last_vector.z) > 1e-6:
            self.pub_nudge.publish(self.last_vector)
            self.get_logger().debug(f"🎮 Nudge: dx={self.last_vector.x:.4f}, dy={self.last_vector.y:.4f}, dz={self.last_vector.z:.4f}")


def main(args=None):
    rclpy.init(args=args)
    node = PS5TeleopRCM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 