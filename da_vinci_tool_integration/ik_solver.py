#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes


VALID_JOINTS = [
    "lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6", "lbr_A7",
    "PSM_outer_roll", "PSM_outer_wrist_pitch", "PSM_outer_wrist_yaw", "PSM_jaw"
]

MIMIC_JOINTS = {
    "PSM_jaw_mimic_1": "PSM_jaw",
    "PSM_jaw_mimic_2": "PSM_jaw"
}


class IKSolver(Node):
    def __init__(self):
        super().__init__('ik_solver')

        self.group_name = "arm"
        self.tip_link = "PSM_tool_virtual_tip"
        self.ik_service = "/compute_ik"
        self.last_joint_state = None

        self.ik_client = self.create_client(GetPositionIK, self.ik_service)
        self.get_logger().info(f"Waiting for {self.ik_service} service...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK service not available.")
            raise RuntimeError("IK service unavailable")

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/tool_target",
            self.pose_callback,
            10
        )

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        #  Add publisher for successful IK poses
        self.success_pose_pub = self.create_publisher(PoseStamped, "/ik_success_pose", 10)
        
        self.timer = self.create_timer(1.0, self.republish_joint_state)
        self.get_logger().info("IK Solver ready. Listening on /tool_target")

    def pose_callback(self, pose_msg):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.tip_link
        req.ik_request.pose_stamped = pose_msg
        req.ik_request.timeout.sec = 2

        # Store the original pose for potential success feedback
        self.current_target_pose = pose_msg

        self.ik_client.call_async(req).add_done_callback(self.handle_ik_response)

    def handle_ik_response(self, future):
        try:
            res = future.result()
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().warn(f"IK failed: error code {res.error_code.val}")
                return

            raw_joint_state = res.solution.joint_state
            filtered = JointState()
            filtered.header.stamp = self.get_clock().now().to_msg()

            joint_map = dict(zip(raw_joint_state.name, raw_joint_state.position))

            for name in VALID_JOINTS:
                if name in joint_map:
                    filtered.name.append(name)
                    filtered.position.append(joint_map[name])

            # Add mimic joints
            for mimic_name, source_name in MIMIC_JOINTS.items():
                if source_name in joint_map:
                    filtered.name.append(mimic_name)
                    filtered.position.append(joint_map[source_name])

            self.js_pub.publish(filtered)
            self.last_joint_state = filtered
            
            #  Publish successful pose for marker feedback
            if hasattr(self, 'current_target_pose'):
                success_pose = PoseStamped()
                success_pose.header.frame_id = self.current_target_pose.header.frame_id
                success_pose.header.stamp = self.get_clock().now().to_msg()
                success_pose.pose = self.current_target_pose.pose
                self.success_pose_pub.publish(success_pose)
                self.get_logger().info(f"Published success to /ik_success_pose: ({success_pose.pose.position.x:.3f}, {success_pose.pose.position.y:.3f}, {success_pose.pose.position.z:.3f})")
            else:
                self.get_logger().warn("current_target_pose not found - cannot publish success feedback")
            
            self.get_logger().info(f"Published joint state with {len(filtered.name)} joints.")

        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")

    def republish_joint_state(self):
        if self.last_joint_state:
            self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.js_pub.publish(self.last_joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 