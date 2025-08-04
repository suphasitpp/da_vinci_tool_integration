#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import pandas as pd
import numpy as np
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from rclpy.time import Time
import os


class BatchPoseTester(Node):
    def __init__(self):
        super().__init__('batch_pose_tester')
        
        # Initialize TF components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for target poses
        self.publisher = self.create_publisher(PoseStamped, '/tool_target', 10)
        
        # Initialize state variables
        self.results = []
        self.current_index = 0
        self.wait_counter = 0
        self.target_pose = None
        
        # Create timer for processing
        self.timer = self.create_timer(0.5, self.process)
        
        # Load poses from Excel file
        self._load_poses()
        
        self.get_logger().info(f"✅ BatchPoseTester initialized with {len(self.poses_df)} poses.")

    def _load_poses(self):
        """Load poses from Excel file"""
        # Try to load from the current working directory first
        current_dir = os.getcwd()
        source_dir = os.path.join(current_dir, 'src', 'da_vinci_tool_integration', 'da_vinci_tool_integration')
        self.get_logger().info(f"🔍 Looking for rounded_poses.xlsx in: {source_dir}")
        
        # Only try to load rounded_poses.xlsx
        excel_path = os.path.join(source_dir, 'rounded_poses.xlsx')
        self.get_logger().info(f"🔍 Checking for file: {excel_path}")
        
        if os.path.exists(excel_path):
            self.poses_df = pd.read_excel(excel_path)
            self.get_logger().info(f"✅ Loaded {len(self.poses_df)} poses from {excel_path}")
        else:
            self.get_logger().error(f"❌ File not found: {excel_path}")
            self.get_logger().error("Please ensure rounded_poses.xlsx exists in the package directory.")
            raise FileNotFoundError(f"rounded_poses.xlsx not found at {excel_path}")



    def process(self):
        """Main processing loop"""
        if self.current_index >= len(self.poses_df):
            self.save_results()
            self.get_logger().info("✅ All poses processed. Results saved.")
            rclpy.shutdown()
            return

        if self.wait_counter == 0:
            # Publish next pose
            self._publish_pose()
            self.wait_counter += 1

        elif self.wait_counter < 3:
            # Wait a few cycles to let TF update
            self.wait_counter += 1

        else:
            # Check TF and compute error
            self._measure_and_record_error()
            # Move to next pose
            self.current_index += 1
            self.wait_counter = 0

    def _publish_pose(self):
        """Publish the current target pose"""
        try:
            row = self.poses_df.iloc[self.current_index]
            
            self.target_pose = PoseStamped()
            self.target_pose.header.frame_id = 'lbr_link_0'
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.target_pose.pose.position.x = row['x']
            self.target_pose.pose.position.y = row['y']
            self.target_pose.pose.position.z = row['z']
            self.target_pose.pose.orientation.x = row['qx']
            self.target_pose.pose.orientation.y = row['qy']
            self.target_pose.pose.orientation.z = row['qz']
            self.target_pose.pose.orientation.w = row['qw']

            self.publisher.publish(self.target_pose)
            self.get_logger().info(f"📤 Published pose {int(row['Pose'])}: ({row['x']:.3f}, {row['y']:.3f}, {row['z']:.3f})")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing pose {self.current_index + 1}: {e}")

    def _measure_and_record_error(self):
        """Measure actual pose and compute errors"""
        try:
            # Lookup TF transform
            tf = self.tf_buffer.lookup_transform('lbr_link_0', 'PSM_tool_virtual_tip', Time())
            t = tf.transform.translation
            r = tf.transform.rotation

            # Compute position error
            actual_pos = np.array([t.x, t.y, t.z])
            target_pos = np.array([
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z
            ])
            pos_error = np.linalg.norm(target_pos - actual_pos) * 1000  # Convert to mm

            # Compute orientation error
            actual_q = np.array([r.x, r.y, r.z, r.w])
            target_q = np.array([
                self.target_pose.pose.orientation.x,
                self.target_pose.pose.orientation.y,
                self.target_pose.pose.orientation.z,
                self.target_pose.pose.orientation.w
            ])
            
            # Normalize quaternions
            actual_q = actual_q / np.linalg.norm(actual_q)
            target_q = target_q / np.linalg.norm(target_q)
            
            # Compute angle error
            dot = np.clip(np.dot(target_q, actual_q), -1.0, 1.0)
            angle_error = 2 * np.arccos(abs(dot)) * 180 / np.pi  # Convert to degrees

            # Record results
            self.results.append({
                "Pose": int(self.poses_df.iloc[self.current_index]['Pose']),
                "x": target_pos[0],
                "y": target_pos[1],
                "z": target_pos[2],
                "qx": target_q[0],
                "qy": target_q[1],
                "qz": target_q[2],
                "qw": target_q[3],
                "Position Error (mm)": pos_error,
                "Orientation Error (deg)": angle_error
            })

            self.get_logger().info(f"✅ Pose {int(self.poses_df.iloc[self.current_index]['Pose'])}: "
                                 f"PosErr = {pos_error:.4f} mm, RotErr = {angle_error:.4f}°")

        except Exception as e:
            self.get_logger().error(f"❌ TF lookup failed for pose {self.current_index + 1}: {e}")
            # Record failed pose with error values
            row = self.poses_df.iloc[self.current_index]
            self.results.append({
                "Pose": int(row['Pose']),
                "x": row['x'],
                "y": row['y'],
                "z": row['z'],
                "qx": row['qx'],
                "qy": row['qy'],
                "qz": row['qz'],
                "qw": row['qw'],
                "Position Error (mm)": float('nan'),
                "Orientation Error (deg)": float('nan')
            })

    def save_results(self):
        """Save results to Excel file with timestamp"""
        try:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = f'ik_error_results_{timestamp}.xlsx'
            self.get_logger().info(f"💾 Saving results to: {output_path}")
            
            df = pd.DataFrame(self.results)
            df.to_excel(output_path, index=False)
            self.get_logger().info(f"✅ Results saved successfully to {output_path}")
            

                
        except Exception as e:
            self.get_logger().error(f"❌ Error saving results: {e}")


def main(args=None):
    """Main function with error handling"""
    try:
        rclpy.init(args=args)
        node = BatchPoseTester()
        
        # Log startup information
        node.get_logger().info("🚀 Starting BatchPoseTester...")
        node.get_logger().info("📋 Will test poses and measure IK accuracy")
        node.get_logger().info("⏱️ Processing at 1Hz (1 second per pose)")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
    finally:
        try:
            rclpy.shutdown()
            print("✅ BatchPoseTester shutdown complete")
        except:
            pass


if __name__ == '__main__':
    main() 