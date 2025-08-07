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
        
        # Initialize joint state with zeros
        self.last_joint_state = {name: 0.0 for name in VALID_JOINTS}
        
        # Track if we've received tool joints from PS5
        self.tool_joints_received = False

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

        self.tool_joint_sub = self.create_subscription(
            JointState,
            "/psm_joint_states",
            self.tool_joint_callback,
            10
        )

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.success_pose_pub = self.create_publisher(PoseStamped, "/ik_success_pose", 10)

        self.create_timer(0.1, self.republish_joint_state)  # 10 Hz is sufficient
        self.get_logger().info("IK Solver ready. Listening on /tool_target and /psm_joint_states")

    def pose_callback(self, pose_msg):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.tip_link
        req.ik_request.pose_stamped = pose_msg
        req.ik_request.timeout.sec = 2

        self.current_target_pose = pose_msg
        self.ik_client.call_async(req).add_done_callback(self.handle_ik_response)

    def tool_joint_callback(self, msg):
        """Preserve tool joints from PS5 control"""
        try:
            for name, pos in zip(msg.name, msg.position):
                if name in VALID_JOINTS and name.startswith("PSM_"):
                    self.last_joint_state[name] = pos
                    self.tool_joints_received = True
            
            # Update mimic joints when tool joints change
            mimic_multipliers = {
                "PSM_jaw_mimic_1": 0.5,
                "PSM_jaw_mimic_2": -0.5
            }
            for mimic, source in MIMIC_JOINTS.items():
                if source in self.last_joint_state:
                    multiplier = mimic_multipliers.get(mimic, 1.0)
                    self.last_joint_state[mimic] = self.last_joint_state[source] * multiplier
            
            self.get_logger().debug(f"Updated tool joints: {dict(zip(msg.name, msg.position))}")
        except Exception as e:
            self.get_logger().error(f"Error processing tool joints: {e}")

    def handle_ik_response(self, future):
        try:
            res = future.result()
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().warn(f"IK failed: error code {res.error_code.val}")
                return

            ik_joint_map = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))

            # Update only arm joints from IK solution
            for name in VALID_JOINTS:
                if name.startswith("lbr_") and name in ik_joint_map:
                    self.last_joint_state[name] = ik_joint_map[name]
                    self.get_logger().debug(f"Updated arm joint {name}: {ik_joint_map[name]}")

            # Add mimic joints with proper multipliers
            mimic_multipliers = {
                "PSM_jaw_mimic_1": 0.5,
                "PSM_jaw_mimic_2": -0.5
            }
            for mimic, source in MIMIC_JOINTS.items():
                if source in self.last_joint_state:
                    multiplier = mimic_multipliers.get(mimic, 1.0)
                    self.last_joint_state[mimic] = self.last_joint_state[source] * multiplier

            # Publish success pose for marker feedback
            if hasattr(self, 'current_target_pose'):
                success_pose = PoseStamped()
                success_pose.header = self.current_target_pose.header
                success_pose.pose = self.current_target_pose.pose
                self.success_pose_pub.publish(success_pose)

        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")

    def republish_joint_state(self):
        """Publish combined joint state (arm from IK + tool from PS5)"""
        if not self.last_joint_state:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.last_joint_state.keys())
        msg.position = [self.last_joint_state[n] for n in msg.name]
        

        
        self.js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 