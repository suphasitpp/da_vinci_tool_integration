#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from std_msgs.msg import Bool  # Add this import at the top

class ToolTargetMarker(Node):
    def __init__(self):
        super().__init__('tool_target_marker')
        self.server = InteractiveMarkerServer(self, "tool_target_marker")
        self.pose_pub = self.create_publisher(PoseStamped, "/tool_target", 10)
        self.rcm_candidate_pub = self.create_publisher(PoseStamped, "/rcm_candidate_pose", 10)

        # Add RCM mode subscription
        self.rcm_mode = False
        self.rcm_mode_sub = self.create_subscription(
            Bool,
            "/rcm_mode",
            self.rcm_mode_callback,
            10
        )

        # Add IK success feedback subscription and pose validation
        self.success_pose_sub = self.create_subscription(
            PoseStamped,
            "/ik_success_pose",
            self.success_pose_callback,
            10
        )
        
        # Add robot state subscription to get current end-effector pose
        self.robot_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.robot_state_callback,
            10
        )
        
        # Initialize with a safe default pose
        self.last_valid_pose = None
        self.waiting_for_ik_response = False
        self.validation_timer = None
        self.ik_timeout_duration = 1.0  # seconds to wait for IK response
        self.marker_initialized = False

        self.marker = InteractiveMarker()
        self.marker.header.frame_id = "lbr_link_0"
        self.marker.name = "tool_target"
        self.marker.description = "Tool Target Pose"
        self.marker.scale = 0.2
        # Pose will be set dynamically based on robot's current position
        # No hardcoded pose - will be set in robot_state_callback

        # Store initial pose as the first valid pose (will be updated when robot state is received)
        self.last_valid_pose = None

        # Add central visual sphere with 3D drag capability
        visual_marker = Marker()
        visual_marker.type = Marker.SPHERE
        visual_marker.scale.x = 0.05
        visual_marker.scale.y = 0.05
        visual_marker.scale.z = 0.05
        visual_marker.color.r = 1.0
        visual_marker.color.g = 0.2
        visual_marker.color.b = 0.2
        visual_marker.color.a = 0.8

        # Combine visual and drag functionality in one control
        drag_control = InteractiveMarkerControl()
        drag_control.name = "move_3d"
        drag_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        drag_control.always_visible = True
        drag_control.markers.append(visual_marker)
        self.marker.controls.append(drag_control)

        # Axis-specific rotation + movement
        for axis in ['x', 'y', 'z']:
            move = InteractiveMarkerControl()
            move.name = f"move_{axis}"
            move.orientation.w = 1.0
            move.orientation.x = 1.0 if axis == 'x' else 0.0
            move.orientation.y = 1.0 if axis == 'y' else 0.0
            move.orientation.z = 1.0 if axis == 'z' else 0.0
            move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            self.marker.controls.append(move)

            rotate = InteractiveMarkerControl()
            rotate.name = f"rotate_{axis}"
            rotate.orientation.w = 1.0
            rotate.orientation.x = 1.0 if axis == 'x' else 0.0
            rotate.orientation.y = 1.0 if axis == 'y' else 0.0
            rotate.orientation.z = 1.0 if axis == 'z' else 0.0
            rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            self.marker.controls.append(rotate)

        self.server.insert(self.marker)
        self.server.setCallback(self.marker.name, self.feedback_callback)
        self.server.applyChanges()
        
        # Validate initial pose by publishing it to IK solver
        self._initial_validation_timer = self.create_timer(0.5, self.validate_initial_pose)  # Small delay to ensure services are ready
        
        self.get_logger().info("Interactive marker ready. Waiting for robot state to initialize position...")

    def robot_state_callback(self, msg):
        """Update marker position based on robot's current end-effector pose"""
        if self.marker_initialized:
            return  # Only initialize once
            
        # Wait a bit for TF to be available
        if not hasattr(self, '_tf_buffer'):
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            return
            
        try:
            # Get the current end-effector pose using TF
            transform = self._tf_buffer.lookup_transform(
                "lbr_link_0", 
                "PSM_tool_virtual_tip", 
                rclpy.time.Time()
            )
            
            # Update marker pose to match robot's current end-effector position
            self.marker.pose.position.x = transform.transform.translation.x
            self.marker.pose.position.y = transform.transform.translation.y
            self.marker.pose.position.z = transform.transform.translation.z
            self.marker.pose.orientation.x = transform.transform.rotation.x
            self.marker.pose.orientation.y = transform.transform.rotation.y
            self.marker.pose.orientation.z = transform.transform.rotation.z
            self.marker.pose.orientation.w = transform.transform.rotation.w
            
            # Update the marker on the server
            self.server.setPose(self.marker.name, self.marker.pose)
            self.server.applyChanges()
            
            # Update last valid pose
            self.last_valid_pose = PoseStamped()
            self.last_valid_pose.header.frame_id = "lbr_link_0"
            self.last_valid_pose.pose = self.marker.pose
            
            self.marker_initialized = True
            self.get_logger().info(f"Marker initialized at robot position: x={self.marker.pose.position.x:.3f}, y={self.marker.pose.position.y:.3f}, z={self.marker.pose.position.z:.3f}")
            
        except Exception as e:
            # Don't spam the logs, just wait for TF to be ready
            pass

    def validate_initial_pose(self):
        """Validate initial pose by sending it to IK solver"""
        if self.last_valid_pose is not None:
            self.pose_pub.publish(self.last_valid_pose)
            self.get_logger().info("Validating initial marker pose with IK solver")
        # Cancel this single-shot timer
        if hasattr(self, '_initial_validation_timer'):
            self._initial_validation_timer.cancel()

    def success_pose_callback(self, pose_msg):
        """Handle successful IK pose feedback"""
        self.last_valid_pose = pose_msg
        self.waiting_for_ik_response = False
        
        # Cancel validation timer since we got a successful response
        if self.validation_timer is not None:
            self.validation_timer.cancel()
            self.validation_timer = None
            
        self.get_logger().debug("Received successful IK pose - pose validated")

    def reset_to_valid_pose(self):
        """Reset marker to last known valid pose"""
        if self.waiting_for_ik_response and self.last_valid_pose is not None:
            self.get_logger().warn("IK failed - resetting marker to last valid pose")
            self.server.setPose(self.marker.name, self.last_valid_pose.pose)
            self.server.applyChanges()
        
        self.waiting_for_ik_response = False
        if self.validation_timer is not None:
            self.validation_timer.cancel()
            self.validation_timer = None

    def rcm_mode_callback(self, msg):
        self.rcm_mode = msg.data
        self.get_logger().info(f"RCM mode updated: {self.rcm_mode}")

    def feedback_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Always publish to RCM candidate topic (for planning)
            pose = PoseStamped()
            pose.header.frame_id = "lbr_link_0"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose = feedback.pose
            self.rcm_candidate_pub.publish(pose)
            
            if self.rcm_mode:
                self.get_logger().info("🔒 RCM mode is active — marker input ignored.")
                return  # Don't send pose to robot
            
            # Publish the new pose to IK solver
            self.pose_pub.publish(pose)
            
            # Start validation timer - if no success response comes back, reset pose
            self.waiting_for_ik_response = True
            
            # Cancel any existing timer
            if self.validation_timer is not None:
                self.validation_timer.cancel()
            
            # Start new validation timer (single-shot)
            self.validation_timer = self.create_timer(
                self.ik_timeout_duration, 
                self.reset_to_valid_pose
            )
            
            self.get_logger().info(f"Published new tool target pose: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = ToolTargetMarker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 