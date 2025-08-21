#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3


class PS5TeleopRCM(Node):
    def __init__(self):
        super().__init__('ps5_teleop_rcm')

        # Parameters
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('scale_xy', 0.01)
        self.declare_parameter('scale_z', 0.04)
        
        self.deadzone = self.get_parameter('deadzone').value
        self.scale_xy = self.get_parameter('scale_xy').value
        self.scale_z = self.get_parameter('scale_z').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.last_vector = Vector3()

        # ROS setup
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_nudge = self.create_publisher(Vector3, '/tool_nudge', 10)

        self.dt = 1.0 / self.publish_rate
        self.create_timer(self.dt, self.send_nudge)

        self.get_logger().info("🎮 PS5 Teleop RCM ready (no tool control)")

    def joy_callback(self, msg):
        # Left stick → XY
        lx = -msg.axes[1]
        ly = -msg.axes[0]
        # Triggers → Z
        l2 = msg.axes[2]
        r2 = msg.axes[5]

        # Apply deadzone
        lx = lx if abs(lx) > self.deadzone else 0.0
        ly = ly if abs(ly) > self.deadzone else 0.0

        dx = self.dt * lx * self.scale_xy
        dy = -self.dt * ly * self.scale_xy
        dz = self.dt * ((1.0 - r2) / 2.0 - (1.0 - l2) / 2.0) * self.scale_z

        self.last_vector = Vector3(x=dx, y=dy, z=dz)

    def send_nudge(self):
        # if any(abs(v) > 1e-6 for v in [self.last_vector.x, self.last_vector.y, self.last_vector.z]):
        self.pub_nudge.publish(self.last_vector)


def main(args=None):
    rclpy.init(args=args)
    node = PS5TeleopRCM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 