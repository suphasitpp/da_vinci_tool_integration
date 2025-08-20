#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class PS5JointController(Node):
    def __init__(self):
        super().__init__("ps5_joint_controller")  # node
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.publisher = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )  # or maybe /joint_trajectory_controller/joint_trajectory (not sure of the topic name)

        self.joystick_axes = [0.0] * 8
        self.joystick_buttons = [0] * 16
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # roll, pitch, jaw_1, jaw_2

        self.timer_period = 0.05  # sec
        self.step_size = 0.8  # deg

        self.timer = self.create_timer(self.timer_period, self.update_joint_commands)

    def joy_callback(self, msg):
        self.joystick_axes = msg.axes
        self.joystick_buttons = msg.buttons

    def update_joint_commands(self):
        # PS5-style mapping
        # L1/R1 buttons for roll
        if self.joystick_buttons[4]:  # L1
            self.joint_positions[0] -= self.step_size  # Roll left
        elif self.joystick_buttons[5]:  # R1
            self.joint_positions[0] += self.step_size  # Roll right
            
        # D-pad up/down for pitch
        if self.joystick_axes[7] > 0.5:  # D-pad up
            self.joint_positions[1] += self.step_size  # Pitch up
        elif self.joystick_axes[7] < -0.5:  # D-pad down
            self.joint_positions[1] -= self.step_size  # Pitch down
            
        # Circle/Square for jaw_1 open/close
        if self.joystick_buttons[1]:  # Circle
            self.joint_positions[2] += self.step_size  # Jaw 1 open
        elif self.joystick_buttons[3]:  # Square
            self.joint_positions[2] -= self.step_size  # Jaw 1 close
            
        # D-pad left/right for jaw_2 open/close
        if self.joystick_axes[6] < -0.5:  # D-pad left
            self.joint_positions[3] += self.step_size  # Jaw 2 open
        elif self.joystick_axes[6] > 0.5:  # D-pad right
            self.joint_positions[3] -= self.step_size  # Jaw 2 close

        # updated positions
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PS5JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 