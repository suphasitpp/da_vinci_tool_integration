#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3, Point
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
        
        # Pose tracking (snap back functionality removed)
        self.last_published_pose = None
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # RCM state
        self.rcm_mode = False
        self.rcm_point = None
        self.last_z_axis = np.array([0, 0, 1])  # Fallback shaft direction for RCM alignment
        
        # GUI Setup
        self.root = tk.Tk()
        self.root.title("RCM Tool Control")

        # Create main frame
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Tool frame offset controls
        ttk.Label(self.frame, text="Tool Frame Offsets:", font=('Arial', 12, 'bold')).grid(row=0, column=0, columnspan=3, pady=(0, 10))

        self.entries = {}
        self.sliders = {}
        
        for i, axis in enumerate(['x', 'y', 'z']):
            # Label
            ttk.Label(self.frame, text=f"{axis.upper()} Offset:").grid(row=i+1, column=0, padx=5, pady=5, sticky=tk.W)
            
            # Entry field
            entry = ttk.Entry(self.frame, width=10)
            entry.insert(0, "0.000")
            entry.grid(row=i+1, column=1, padx=5, pady=5)
            self.entries[axis] = entry

            # Slider
            slider = tk.Scale(self.frame, from_=-0.10, to=0.10, resolution=0.001, 
                             orient=tk.HORIZONTAL, length=150,
                             command=lambda val, ax=axis: self.update_entry_from_slider(ax, val))
            slider.set(0.0)
            slider.grid(row=i+1, column=2, padx=5, pady=5)
            self.sliders[axis] = slider

        # Buttons
        self.send_button = ttk.Button(self.frame, text="🚀 Send RCM-Constrained Pose", 
                                     command=self.send_rcm_pose)
        self.send_button.grid(row=4, column=0, columnspan=3, pady=10)

        self.set_rcm_button = ttk.Button(self.frame, text="📍 Set RCM Point", 
                                        command=self.set_rcm_point)
        self.set_rcm_button.grid(row=5, column=0, columnspan=3, pady=5)

        self.toggle_button = ttk.Button(self.frame, text="🔄 Toggle RCM Mode", 
                                       command=self.toggle_rcm_mode)
        self.toggle_button.grid(row=6, column=0, columnspan=3, pady=5)

        # Status display
        self.status_label = ttk.Label(self.frame, text="RCM Mode: OFF", foreground="red", 
                                     font=('Arial', 10, 'bold'))
        self.status_label.grid(row=7, column=0, columnspan=3, pady=10)

        # RCM point info
        self.rcm_info_label = ttk.Label(self.frame, text="RCM Point: Not Set", 
                                       foreground="gray")
        self.rcm_info_label.grid(row=8, column=0, columnspan=3, pady=5)
        
        # PS5 Input display
        self.nudge_label = ttk.Label(self.frame, text="🎮 PS5 Input: X=0.000 Y=0.000 Z=0.000", 
                                    foreground="gray")
        self.nudge_label.grid(row=9, column=0, columnspan=3, pady=5)
        
        # Pose tracking
        self.last_pose_time = None

    def rcm_mode_callback(self, msg):
        """Handle external RCM mode changes"""
        self.rcm_mode = msg.data
        self.update_status_display()

    def rcm_pose_callback(self, msg):
        """Handle RCM point updates"""
        # Only accept RCM updates when RCM mode is OFF (before setting RCM)
        if self.rcm_mode:
            self.get_logger().debug("🔒 RCM mode is ON - ignoring pose updates (RCM point is locked)")
            return  # RCM mode is ON — ignore updates to keep RCM point frozen
            
        # Only update RCM point, don't capture tool poses yet
        self.rcm_point = msg.pose
        self.update_rcm_info()
        self.get_logger().debug(
            f"📍 RCM candidate updated: ({self.rcm_point.position.x:.3f}, "
            f"{self.rcm_point.position.y:.3f}, {self.rcm_point.position.z:.3f})"
        )

    def update_entry_from_slider(self, axis, val):
        """Update entry field when slider changes"""
        self.entries[axis].delete(0, tk.END)
        self.entries[axis].insert(0, f"{float(val):.3f}")

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
        """Set RCM point from current interactive marker position and enable RCM mode"""
        # First, send trigger to get current RCM candidate pose
        msg = Bool()
        msg.data = True
        self.rcm_trigger_pub.publish(msg)
        self.get_logger().info("📡 Sent RCM trigger request")
        
        # Wait a moment for the RCM pose to be updated
        self.root.after(100, self._complete_rcm_setup)
    
    def _complete_rcm_setup(self):
        """Complete RCM setup after pose is captured"""
        if self.rcm_point is not None:
            # Enable RCM mode and capture tool tip pose
            self.rcm_mode = True
            if self.capture_tool_tip_pose():
                self.get_logger().info("✅ RCM point set and tool tip pose captured")
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

    def apply_rcm_motion(self, dx, dy, dz):
        """Apply a joystick-driven offset to the tool tip with RCM constraint."""
        if not self.rcm_mode or self.rcm_point is None:
            self.get_logger().warn("RCM motion skipped: RCM mode not active or point not set.")
            return

        # Check if virtual tip is initialized
        if self.virtual_tip_pos is None or self.virtual_tip_rot is None:
            self.get_logger().warn("Virtual tip not initialized - please set RCM first.")
            return

        # Filter small joystick input (deadzone)
        def clip(val): return 0.0 if abs(val) < 1e-4 else val
        dx = clip(dx)
        dy = clip(dy)
        dz = clip(dz)

        # Build local tool frame (X/Y/Z axes)
        x_axis = self.virtual_tip_rot[:, 0]
        y_axis = self.virtual_tip_rot[:, 1]
        z_axis = self.virtual_tip_rot[:, 2]

        # Apply local offset to tip
        tip_pos = self.virtual_tip_pos + dx * x_axis + dy * y_axis + dz * z_axis

        # Reuse cached orientation if no motion
        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            rot_matrix = self.virtual_tip_rot
            quat = R.from_matrix(rot_matrix).as_quat()
        else:
            # Compute new Z-axis (shaft direction) pointing to RCM
            rcm_vec = np.array([
                self.rcm_point.position.x,
                self.rcm_point.position.y,
                self.rcm_point.position.z
            ])
            shaft_vec = rcm_vec - tip_pos

            if np.linalg.norm(shaft_vec) < 1e-4:
                z_axis = self.last_z_axis
            else:
                z_axis = normalize(shaft_vec)
                self.last_z_axis = z_axis

            # Compute orthogonal axes
            up = np.array([0, 0, 1]) if abs(np.dot(z_axis, [0, 0, 1])) < 0.95 else np.array([0, 1, 0])
            x_axis_new = normalize(np.cross(up, z_axis))
            y_axis_new = np.cross(z_axis, x_axis_new)

            rot_matrix = np.column_stack((x_axis_new, y_axis_new, z_axis))
            quat = R.from_matrix(rot_matrix).as_quat()

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

        # Update virtual tip state
        self.virtual_tip_pos = tip_pos
        self.virtual_tip_rot = rot_matrix

        # Publish debug marker
        self.publish_target_pose_frame(tip_pos, rot_matrix, False)
        self.publish_actual_pose_frame()

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

    def send_rcm_pose(self):
        """Send RCM-constrained pose with GUI offsets"""
        
        # 🔐 Safety checks
        if not self.rcm_mode:
            self.get_logger().warn("❗ RCM Mode is OFF. Cannot send pose.")
            return

        if self.rcm_point is None:
            self.get_logger().error("❌ No RCM point available. Please set RCM first.")
            return

        # ✅ CAPTURE EXACT TOOL TIP POSE when RCM is set
        if not self.capture_tool_tip_pose():
            self.get_logger().error("❌ Failed to capture tool tip pose")
            return

        # ✅ PUBLISH EXACT POSE to /tool_target so target matches actual tip
        exact_pose = PoseStamped()
        exact_pose.header.frame_id = "lbr_link_0"
        exact_pose.header.stamp = self.get_clock().now().to_msg()
        exact_pose.pose.position.x = self.tool_tip_position[0]
        exact_pose.pose.position.y = self.tool_tip_position[1]
        exact_pose.pose.position.z = self.tool_tip_position[2]
        exact_pose.pose.orientation.x = self.tool_tip_orientation[0]
        exact_pose.pose.orientation.y = self.tool_tip_orientation[1]
        exact_pose.pose.orientation.z = self.tool_tip_orientation[2]
        exact_pose.pose.orientation.w = self.tool_tip_orientation[3]
        
        self.publisher_.publish(exact_pose)
        self.get_logger().info("✅ Published exact tool tip pose to align target with actual tip")

        # Get tool frame offsets from GUI
        try:
            x_offset = float(self.entries['x'].get())
            y_offset = float(self.entries['y'].get())
            z_offset = float(self.entries['z'].get())
        except ValueError:
            self.get_logger().error("❌ Invalid input. Please use numeric values for offsets.")
            return

        # Get RCM in base frame
        rcm_pos = self.rcm_point_vec()

        try:
            # Use exact captured position and orientation
            tip_origin = self.tool_tip_position.copy()
            quat = self.tool_tip_orientation.copy()
            rot = R.from_quat(quat)
            rot_matrix = rot.as_matrix()

            # Local frame axes from captured tool orientation
            z_axis = rot_matrix[:, 2]  # tool shaft direction
            x_axis = rot_matrix[:, 0]
            y_axis = rot_matrix[:, 1]

            # Apply offset in tool's local frame
            tip_pos = tip_origin + x_offset * x_axis + y_offset * y_axis + z_offset * z_axis

            # Recalculate Z-axis to point from tip to RCM (RCM constraint)
            new_z_axis = normalize(rcm_pos - tip_pos)

            # Recompute X and Y axes
            if abs(np.dot(new_z_axis, [0, 0, 1])) > 0.95:
                up = np.array([0, 1, 0])
            else:
                up = np.array([0, 0, 1])
            new_x_axis = normalize(np.cross(up, new_z_axis))
            new_y_axis = np.cross(new_z_axis, new_x_axis)

            # Final orientation
            rot_matrix = np.column_stack((new_x_axis, new_y_axis, new_z_axis))
            quat = R.from_matrix(rot_matrix).as_quat()

        except Exception as e:
            self.get_logger().error(f"❌ Failed to compute tool pose: {e}")
            return

        # 📡 Publish Pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "lbr_link_0"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = tip_pos[0]
        pose_msg.pose.position.y = tip_pos[1]
        pose_msg.pose.position.z = tip_pos[2]
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.publisher_.publish(pose_msg)

        self.get_logger().info(f"✅ Sent RCM-constrained pose with offsets: x={x_offset:.3f}, y={y_offset:.3f}, z={z_offset:.3f}")
        self.get_logger().info(f"📍 Tip position: ({tip_pos[0]:.3f}, {tip_pos[1]:.3f}, {tip_pos[2]:.3f}) | RCM: ({rcm_pos[0]:.3f}, {rcm_pos[1]:.3f}, {rcm_pos[2]:.3f})")

    def spin_ros(self):
        """Non-blocking ROS spinning"""
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.spin_ros)

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
