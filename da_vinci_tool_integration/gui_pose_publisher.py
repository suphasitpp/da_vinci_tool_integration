#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from std_msgs.msg import Bool
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker


def normalize(v):
    """Normalize a vector"""
    norm = np.linalg.norm(v)
    if norm < 1e-6:
        raise ValueError("Cannot normalize zero vector")
    return v / norm


class PosePublisherGUI(Node):
    def __init__(self):
        super().__init__('pose_publisher_gui')

        # Publishers
        self.publisher_ = self.create_publisher(PoseStamped, '/tool_target', 10)
        self.rcm_mode_pub = self.create_publisher(Bool, '/rcm_mode', 10)
        self.rcm_trigger_pub = self.create_publisher(Bool, '/set_rcm_trigger', 10)
        self.arrow_pub = self.create_publisher(Marker, '/tip_visual_marker', 10)

        # Subscribers
        self.rcm_mode_sub = self.create_subscription(Bool, '/rcm_mode', self.rcm_mode_callback, 10)
        self.rcm_pose_sub = self.create_subscription(PoseStamped, '/rcm_candidate_pose', self.rcm_pose_callback, 10)
        self.nudge_sub = self.create_subscription(Vector3, '/tool_nudge', self.nudge_callback, 10)

        # State variables
        self.rcm_point = None
        self.rcm_mode = False
        self.tool_tip_position = None  # Store actual tool tip position separately from RCM
        self.tool_tip_orientation = None  # Store actual tool tip orientation
        
        # Virtual tip state (for PS5 motion)
        self.virtual_tip_pos = None
        self.virtual_tip_rot = None
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # RCM state
        self.rcm_mode = False
        self.rcm_point = None
        
        # RCM alignment tolerance for stability
        self.RCM_ALIGNMENT_TOLERANCE = 0.002  # 2mm alignment tolerance
        
        # GUI Setup
        self.root = tk.Tk()
        self.root.title("RCM Tool Control")

        # Create main frame
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Buttons
        self.set_rcm_button = ttk.Button(self.frame, text="📍 Set RCM Point", 
                                        command=self.set_rcm_point)
        self.set_rcm_button.grid(row=0, column=0, columnspan=3, pady=5)

        self.toggle_button = ttk.Button(self.frame, text="🔄 Toggle RCM Mode", 
                                       command=self.toggle_rcm_mode)
        self.toggle_button.grid(row=1, column=0, columnspan=3, pady=5)

        # Status display
        self.status_label = ttk.Label(self.frame, text="RCM Mode: OFF", foreground="red", 
                                     font=('Arial', 10, 'bold'))
        self.status_label.grid(row=2, column=0, columnspan=3, pady=10)

        # RCM point info
        self.rcm_info_label = ttk.Label(self.frame, text="RCM Point: Not Set", 
                                       foreground="gray")
        self.rcm_info_label.grid(row=3, column=0, columnspan=3, pady=5)
        
        # PS5 Input display
        self.nudge_label = ttk.Label(self.frame, text="🎮 PS5 Input: X=0.000 Y=0.000 Z=0.000", 
                                    foreground="gray")
        self.nudge_label.grid(row=4, column=0, columnspan=3, pady=5)
        
    def rcm_mode_callback(self, msg):
        """Handle external RCM mode changes"""
        self.rcm_mode = msg.data
        self.update_status_display()

    def rcm_pose_callback(self, msg):
        """Handle RCM point updates (legacy - now gets from TF)"""
        # This is kept for backward compatibility but RCM point is now set via TF
        pass

    def update_status_display(self):
        """Update GUI status labels"""
        if self.rcm_mode:
            self.status_label.config(text="RCM Mode: ON", foreground="green")
        else:
            self.status_label.config(text="RCM Mode: OFF", foreground="red")

    def update_rcm_info(self):
        """Update RCM point information display"""
        if self.rcm_point:
            if self.rcm_mode:
                # RCM point is locked (RCM mode is active)
                self.rcm_info_label.config(
                    text=f"🔒 RCM Point (LOCKED): ({self.rcm_point.position.x:.3f}, "
                         f"{self.rcm_point.position.y:.3f}, {self.rcm_point.position.z:.3f})",
                    foreground="green"
                )
            else:
                # RCM point is set but not locked
                self.rcm_info_label.config(
                    text=f"📍 RCM Point: ({self.rcm_point.position.x:.3f}, "
                         f"{self.rcm_point.position.y:.3f}, {self.rcm_point.position.z:.3f})",
                    foreground="blue"
                )
        else:
            self.rcm_info_label.config(text="RCM Point: Not Set", foreground="gray")

    def set_rcm_point(self):
        """Set RCM point from current tool tip position via TF and enable RCM mode"""
        try:
            # Get current tool tip position from TF
            transform = self.tf_buffer.lookup_transform(
                "lbr_link_0", 
                "PSM_tool_virtual_tip", 
                rclpy.time.Time()
            )
            
            # Set RCM point to current tool position
            self.rcm_point = Pose()
            self.rcm_point.position.x = transform.transform.translation.x
            self.rcm_point.position.y = transform.transform.translation.y
            self.rcm_point.position.z = transform.transform.translation.z
            self.rcm_point.orientation.x = transform.transform.rotation.x
            self.rcm_point.orientation.y = transform.transform.rotation.y
            self.rcm_point.orientation.z = transform.transform.rotation.z
            self.rcm_point.orientation.w = transform.transform.rotation.w
            
            self.get_logger().info(f"📍 RCM point set from TF: x={self.rcm_point.position.x:.3f}, y={self.rcm_point.position.y:.3f}, z={self.rcm_point.position.z:.3f}")
            
            # Send trigger to RCM Manager
            msg = Bool()
            msg.data = True
            self.rcm_trigger_pub.publish(msg)
            
            # Complete RCM setup
            self._complete_rcm_setup()
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to get RCM point from TF: {e}")
            self.get_logger().error("Make sure the robot is running and TF is available")
    
    def _complete_rcm_setup(self):
        """Complete RCM setup after pose is captured"""
        if self.rcm_point is not None:
            # Enable RCM mode and capture tool tip pose
            self.rcm_mode = True
            if self.capture_tool_tip_pose():
                self.get_logger().info("✅ RCM point set and tool tip pose captured")

                # 🚀 Publish markers so actual and target poses are immediately visible and aligned
                self.publish_target_pose_frame(self.virtual_tip_pos, self.virtual_tip_rot, ik_failed=False)
                self.publish_actual_pose_frame()
            else:
                self.get_logger().warn("⚠️ RCM point set but failed to capture tool tip pose")
            
            # Publish RCM mode status
            msg = Bool()
            msg.data = self.rcm_mode
            self.rcm_mode_pub.publish(msg)
            self.update_status_display()
            self.update_rcm_info()
        else:
            self.get_logger().warn("⚠️ RCM point not available - please try again")

    def toggle_rcm_mode(self):
        """Toggle RCM mode on/off"""
        self.rcm_mode = not self.rcm_mode
        
        if self.rcm_mode:
            # RCM mode enabled - capture tool tip pose for PS5 motion
            if self.rcm_point is not None:
                if self.capture_tool_tip_pose():
                    self.get_logger().info("✅ RCM mode enabled - tool tip pose captured for PS5 motion")
                else:
                    self.get_logger().warn("⚠️ RCM mode enabled but failed to capture tool tip pose")
            else:
                self.get_logger().warn("⚠️ RCM mode enabled but no RCM point set")
        else:
            # 🔄 RCM RESET: Clear RCM point when disabling RCM mode
            self.rcm_point = None
            self.tool_tip_position = None
            self.tool_tip_orientation = None
            self.virtual_tip_pos = None
            self.virtual_tip_rot = None
            self.update_rcm_info()
            self.get_logger().info("🔄 RCM mode disabled - RCM point cleared")
        
        msg = Bool()
        msg.data = self.rcm_mode
        self.rcm_mode_pub.publish(msg)
        self.update_status_display()
        self.get_logger().info(f"🔄 RCM mode {'enabled' if self.rcm_mode else 'disabled'}")

    def rcm_point_vec(self):
        """Helper function to get RCM point as numpy array"""
        return np.array([
            self.rcm_point.position.x,
            self.rcm_point.position.y,
            self.rcm_point.position.z
        ])

    def verify_ik_reached(self, target_pos):
        """Check if robot reached the target tip position, using RCM alignment tolerance instead of distance"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'lbr_link_0', 'PSM_tool_virtual_tip', rclpy.time.Time()
            )
            pos = transform.transform.translation
            actual = np.array([pos.x, pos.y, pos.z])

            # Compute Z-axis (shaft direction) from published orientation
            ori = transform.transform.rotation
            quat = [ori.x, ori.y, ori.z, ori.w]
            rot_matrix = R.from_quat(quat).as_matrix()
            z_axis = rot_matrix[:, 2]

            # Check alignment error using perpendicular distance
            rcm_vec = self.rcm_point_vec()
            shaft_error = np.linalg.norm(np.cross(rcm_vec - actual, z_axis))

            if shaft_error > self.RCM_ALIGNMENT_TOLERANCE:
                self.get_logger().warn(
                    f"⚠️ IK reached pose, but shaft alignment error is {shaft_error*1000:.2f} mm — exceeds RCM tolerance"
                )
                # Optionally: show red marker but do NOT snap back
                self.publish_target_pose_frame(actual, rot_matrix, ik_failed=True)
            else:
                self.get_logger().debug("✅ Shaft alignment verified — IK success.")

        except Exception as e:
            self.get_logger().warn(f"⚠️ IK verification failed: {e}")

    def apply_rcm_motion(self, dx, dy, dz):
        """Apply a joystick-driven offset to the tool tip with RCM constraint."""
        if not self.rcm_mode or self.rcm_point is None:
            self.get_logger().warn("RCM motion skipped: RCM mode not active or point not set.")
            return

        if self.virtual_tip_pos is None or self.virtual_tip_rot is None:
            self.get_logger().warn("Virtual tip not initialized - please set RCM first.")
            return

        # Deadzone filtering
        def clip(val): return 0.0 if abs(val) < 1e-4 else val
        dx, dy, dz = clip(dx), clip(dy), clip(dz)

        # Use current tool frame
        x_axis = self.virtual_tip_rot[:, 0]
        y_axis = self.virtual_tip_rot[:, 1]
        z_axis = self.virtual_tip_rot[:, 2]

        # Apply offset in local frame
        tip_pos = self.virtual_tip_pos + dx * x_axis + dy * y_axis + dz * z_axis

        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            rot_matrix = self.virtual_tip_rot
            quat = R.from_matrix(rot_matrix).as_quat()
        else:
            # Step 1: compute new shaft direction
            rcm_vec = self.rcm_point_vec()
            z_axis = normalize(tip_pos - rcm_vec)

            # Step 2: reproject original X to maintain orientation
            x_original = self.virtual_tip_rot[:, 0]
            x_axis = x_original - np.dot(x_original, z_axis) * z_axis
            x_axis = normalize(x_axis)
            y_axis = np.cross(z_axis, x_axis)

            rot_matrix = np.column_stack((x_axis, y_axis, z_axis))
            quat = R.from_matrix(rot_matrix).as_quat()

            # Optional: alignment check
            alignment_error = np.linalg.norm(np.cross(rcm_vec - tip_pos, z_axis))
            if alignment_error > self.RCM_ALIGNMENT_TOLERANCE:
                self.get_logger().warn(f"⚠️ Shaft misaligned by {alignment_error*1000:.2f} mm")
            else:
                self.get_logger().debug(f"✅ Shaft alignment OK: {alignment_error*1000:.2f} mm")

        # Publish pose
        pose = PoseStamped()
        pose.header.frame_id = "lbr_link_0"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tip_pos[0]
        pose.pose.position.y = tip_pos[1]
        pose.pose.position.z = tip_pos[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.publisher_.publish(pose)

        # Update state
        self.virtual_tip_pos = tip_pos
        self.virtual_tip_rot = rot_matrix

        # RViz debug markers
        self.publish_target_pose_frame(tip_pos, rot_matrix, ik_failed=False)
        self.publish_actual_pose_frame()

        # IK check (delayed)
        self.root.after(200, lambda: self.verify_ik_reached(tip_pos))

        self.get_logger().info(
            f"🎯 RCM Motion: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f} → "
            f"Tip=({tip_pos[0]:.3f}, {tip_pos[1]:.3f}, {tip_pos[2]:.3f})"
        )

    def publish_target_pose_frame(self, tip_pos, rot_matrix, ik_failed=False):
        """Publish 3 arrow markers showing the target pose coordinate frame."""
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        
        origin = np.array(tip_pos)
        axes = {
            "x": (rot_matrix[:, 0], [1.0, 0.0, 0.0]),  # red
            "y": (rot_matrix[:, 1], [0.0, 1.0, 0.0]),  # green
            "z": (rot_matrix[:, 2], [0.0, 0.0, 1.0])   # blue
        }

        for i, (name, (axis, color)) in enumerate(axes.items()):
            marker = Marker()
            marker.header.frame_id = "lbr_link_0"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "target_tip_axes"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.01  # shaft thickness
            marker.scale.y = 0.02  # head width
            marker.scale.z = 0.0   # not used for ARROW
            
            # Color based on IK status
            if ik_failed:
                # Red markers when IK fails (warning color)
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            else:
                # Light/transparent colors for target pose (commanded)
                light = 0.4  # brightness base
                marker.color = ColorRGBA(
                    r=light + 0.6 * color[0],
                    g=light + 0.6 * color[1],
                    b=light + 0.6 * color[2],
                    a=0.5  # semi-transparent
                )
            
            marker.lifetime.sec = 0  # persistent until overwritten

            start = origin
            end = origin + 0.05 * axis  # 5cm axis line

            marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
            marker.points.append(Point(x=end[0], y=end[1], z=end[2]))

            self.arrow_pub.publish(marker)

    def publish_actual_pose_frame(self):
        """Publish 3 arrow markers showing the actual robot tip pose from TF."""
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'lbr_link_0', 'PSM_tool_virtual_tip', rclpy.time.Time()
            )
            pos = transform.transform.translation
            ori = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            q = [ori.x, ori.y, ori.z, ori.w]
            rot = R.from_quat(q)
            rot_matrix = rot.as_matrix()
            
            origin = np.array([pos.x, pos.y, pos.z])
            axes = {
                "x": (rot_matrix[:, 0], [1.0, 0.0, 0.0]),  # red
                "y": (rot_matrix[:, 1], [0.0, 1.0, 0.0]),  # green
                "z": (rot_matrix[:, 2], [0.0, 0.0, 1.0])   # blue
            }

            for i, (name, (axis, color)) in enumerate(axes.items()):
                marker = Marker()
                marker.header.frame_id = "lbr_link_0"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "actual_tip_axes"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.x = 0.01  # shaft thickness
                marker.scale.y = 0.02  # head width
                marker.scale.z = 0.0   # not used for ARROW
                marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
                marker.lifetime.sec = 0  # persistent until overwritten

                start = origin
                end = origin + 0.05 * axis  # 5cm axis line

                marker.points.append(Point(x=start[0], y=start[1], z=start[2]))
                marker.points.append(Point(x=end[0], y=end[1], z=end[2]))

                self.arrow_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f"❌ Could not publish actual tip marker: {e}")

    def nudge_callback(self, msg):
        """Receive motion deltas from teleop node and apply RCM-constrained motion."""
        dx, dy, dz = msg.x, msg.y, msg.z
        
        # Update PS5 input display
        self.nudge_label.config(
            text=f"🎮 PS5 Input: X={dx:+.3f}  Y={dy:+.3f}  Z={dz:+.3f}",
            foreground="blue"
        )
        
        # Debug: Log PS5 input values
        if abs(dx) > 0.001 or abs(dy) > 0.001 or abs(dz) > 0.001:
            self.get_logger().info(f"🎮 PS5 motion detected: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}")
        
        self.apply_rcm_motion(dx, dy, dz)

    def capture_tool_tip_pose(self):
        """Capture the exact tool tip pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'lbr_link_0',  # target frame (base)
                'PSM_tool_virtual_tip',  # source frame (tool tip)
                rclpy.time.Time()
            )
            
            # Capture exact position and orientation
            self.tool_tip_position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Capture exact orientation
            quat = transform.transform.rotation
            self.tool_tip_orientation = np.array([quat.x, quat.y, quat.z, quat.w])
            
            # Initialize virtual tip with exact captured RCM pose
            self.virtual_tip_pos = self.tool_tip_position.copy()
            self.virtual_tip_rot = R.from_quat(self.tool_tip_orientation).as_matrix()
            
            self.get_logger().info(f"✅ Captured tool tip pose: ({self.tool_tip_position[0]:.3f}, {self.tool_tip_position[1]:.3f}, {self.tool_tip_position[2]:.3f})")
            return True
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not capture tool tip pose: {e}")
            self.tool_tip_position = None
            self.tool_tip_orientation = None
            return False

    def spin_ros(self):
        """Non-blocking ROS spinning"""
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self.spin_ros)

    def run(self):
        """Start the GUI and ROS spinning"""
        self.root.after(100, self.spin_ros)
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    gui = PosePublisherGUI()
    gui.run()
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
