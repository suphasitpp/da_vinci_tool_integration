#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Vector3


class PS5TeleopRCM(Node):
    def __init__(self):
        super().__init__('ps5_teleop_rcm')

        # Parameters
        self.deadzone = 0.1
        self.scale_xy = 0.002  # meters per step (reduced for slower movement)
        self.scale_z = 0.002
        self.psm_step = 0.05   # radians per button press (increased for faster movement)
        self.publish_rate = 20.0  # Hz

        # Tool joint positions
        self.psm_joint_positions = {
            "PSM_outer_roll": 0.0,
            "PSM_outer_wrist_pitch": 0.0,
            "PSM_outer_wrist_yaw": 0.0,
            "PSM_jaw": 0.0
        }

        self.psm_joint_limits = {
            "PSM_outer_roll": (-4.53786, 4.53786),  # ±260° from URDF
            "PSM_outer_wrist_pitch": (-1.5708, 1.5708),  # ±90° from URDF
            "PSM_outer_wrist_yaw": (-1.3963, 1.3963),  # ±80° from URDF
            "PSM_jaw": (0.0, 1.5708)  # 0° to 90° from URDF
        }

        self.last_vector = Vector3()

        # ROS setup
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_nudge = self.create_publisher(Vector3, '/tool_nudge', 10)
        self.pub_psm_joints = self.create_publisher(JointState, '/psm_joint_states', 10)

        self.create_timer(1.0 / self.publish_rate, self.send_nudge)
        self.create_timer(1.0 / self.publish_rate, self.publish_tool_joints)

        self.get_logger().info("🎮 PS5 Teleop RCM ready with tool joint control")

    def joy_callback(self, msg):
        # Left stick → XY
        lx = -msg.axes[1]
        ly = -msg.axes[0]
        # Triggers → Z
        l2 = msg.axes[2]
        r2 = msg.axes[5]

        dx = lx * self.scale_xy if abs(lx) > self.deadzone else 0.0
        dy = -ly * self.scale_xy if abs(ly) > self.deadzone else 0.0
        dz = ((1.0 - r2) / 2.0 - (1.0 - l2) / 2.0) * self.scale_z

        self.last_vector = Vector3(x=dx, y=dy, z=dz)

        self.handle_tool_joints(msg)

    def handle_tool_joints(self, msg):
        # L1/R1 → roll
        if msg.buttons[4]:  # L1
            self.update_joint("PSM_outer_roll", -self.psm_step)
        if msg.buttons[5]:  # R1
            self.update_joint("PSM_outer_roll", self.psm_step)

        # D-pad pitch/yaw (inverted)
        if msg.axes[7] > 0.5:  # D-pad up
            self.update_joint("PSM_outer_wrist_pitch", -self.psm_step)
        if msg.axes[7] < -0.5:  # D-pad down
            self.update_joint("PSM_outer_wrist_pitch", self.psm_step)
        if msg.axes[6] < -0.5:  # D-pad left
            self.update_joint("PSM_outer_wrist_yaw", self.psm_step)
        if msg.axes[6] > 0.5:  # D-pad right
            self.update_joint("PSM_outer_wrist_yaw", -self.psm_step)

        # Square / Circle → jaw
        if msg.buttons[3]:  # Square
            self.update_joint("PSM_jaw", -self.psm_step)
        if msg.buttons[1]:  # Circle
            self.update_joint("PSM_jaw", self.psm_step)

        # Triangle → Reset all tool joints to zero
        if msg.buttons[2]:  # Triangle
            self.reset_tool_joints()

    def update_joint(self, name, delta):
        min_limit, max_limit = self.psm_joint_limits[name]
        new_val = self.psm_joint_positions[name] + delta
        self.psm_joint_positions[name] = max(min(new_val, max_limit), min_limit)

    def send_nudge(self):
        if any(abs(v) > 1e-6 for v in [self.last_vector.x, self.last_vector.y, self.last_vector.z]):
            self.pub_nudge.publish(self.last_vector)

    def publish_tool_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.psm_joint_positions.keys())
        msg.position = list(self.psm_joint_positions.values())
        self.pub_psm_joints.publish(msg)

    def reset_tool_joints(self):
        """Reset all tool joints to zero position"""
        for joint_name in self.psm_joint_positions:
            self.psm_joint_positions[joint_name] = 0.0
        self.get_logger().info("🎮 Tool joints reset to home position")


def main(args=None):
    rclpy.init(args=args)
    node = PS5TeleopRCM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 