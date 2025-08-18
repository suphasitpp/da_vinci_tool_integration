#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Vector3


class PS5TeleopRCM(Node):
    def __init__(self):
        super().__init__('ps5_teleop_rcm')

        # Parameters
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('scale_xy', 0.0004)
        self.declare_parameter('scale_z', 0.0004)
        self.declare_parameter('psm_step', 0.03)
        self.declare_parameter('psm_velocity_scale', 0.5)  # rad/s for smooth movement
        
        self.deadzone = self.get_parameter('deadzone').value
        self.scale_xy = self.get_parameter('scale_xy').value
        self.scale_z = self.get_parameter('scale_z').value
        self.psm_step = self.get_parameter('psm_step').value
        self.psm_velocity_scale = self.get_parameter('psm_velocity_scale').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Tool joint positions
        self.psm_joint_positions = {
            "PSM_outer_roll": 0.0,
            "PSM_outer_wrist_pitch": 0.0,
            "PSM_outer_wrist_yaw": 0.0,
            "PSM_jaw": 0.0
        }
        
        # Tool joint velocities for smooth movement
        self.psm_joint_velocities = {
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
        # L1/R1 → roll (hold-to-move)
        if msg.buttons[4]:  # L1
            self.psm_joint_velocities["PSM_outer_roll"] = -self.psm_velocity_scale
        elif msg.buttons[5]:  # R1
            self.psm_joint_velocities["PSM_outer_roll"] = self.psm_velocity_scale
        else:
            self.psm_joint_velocities["PSM_outer_roll"] = 0.0

        # D-pad pitch/yaw (hold-to-move)
        if msg.axes[7] > 0.5:  # D-pad up
            self.psm_joint_velocities["PSM_outer_wrist_pitch"] = -self.psm_velocity_scale
        elif msg.axes[7] < -0.5:  # D-pad down
            self.psm_joint_velocities["PSM_outer_wrist_pitch"] = self.psm_velocity_scale
        else:
            self.psm_joint_velocities["PSM_outer_wrist_pitch"] = 0.0
            
        if msg.axes[6] < -0.5:  # D-pad left
            self.psm_joint_velocities["PSM_outer_wrist_yaw"] = self.psm_velocity_scale
        elif msg.axes[6] > 0.5:  # D-pad right
            self.psm_joint_velocities["PSM_outer_wrist_yaw"] = -self.psm_velocity_scale
        else:
            self.psm_joint_velocities["PSM_outer_wrist_yaw"] = 0.0

        # Square / Circle → jaw (hold-to-move)
        if msg.buttons[3]:  # Square
            self.psm_joint_velocities["PSM_jaw"] = -self.psm_velocity_scale
        elif msg.buttons[1]:  # Circle
            self.psm_joint_velocities["PSM_jaw"] = self.psm_velocity_scale
        else:
            self.psm_joint_velocities["PSM_jaw"] = 0.0

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
        # Update positions based on velocities (smooth movement)
        dt = 1.0 / self.publish_rate  # Time step
        
        for joint_name in self.psm_joint_positions:
            # Update position based on velocity
            new_pos = self.psm_joint_positions[joint_name] + self.psm_joint_velocities[joint_name] * dt
            
            # Apply joint limits
            min_limit, max_limit = self.psm_joint_limits[joint_name]
            self.psm_joint_positions[joint_name] = max(min(new_pos, max_limit), min_limit)
            
            # Stop velocity if we hit limits
            if (new_pos <= min_limit and self.psm_joint_velocities[joint_name] < 0) or \
               (new_pos >= max_limit and self.psm_joint_velocities[joint_name] > 0):
                self.psm_joint_velocities[joint_name] = 0.0
        
        # Publish joint state
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