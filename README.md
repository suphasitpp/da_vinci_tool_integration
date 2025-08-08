# da_vinci_tool_integration

This ROS 2 package provides surgical robotics simulation combining KUKA LBR Med7 arm with da Vinci PSM tools. Includes kinematic solvers, RCM management, and surgical workspace visualization.

---

## Features

- **Robot Integration**: LBR Med7 + da Vinci PSM + custom adaptor
- **Kinematic Solvers**: Forward/inverse kinematics for motion planning
- **RCM Management**: Remote Center of Motion constraint handling
- **Surgical Visualization**: Static surgical box with reference markers
- **Interactive Control**: Interactive target marker for robot positioning
- **PS5 Teleoperation**: PlayStation 5 controller support

---

## Dependencies

You must have the following packages in your ROS 2 workspace:

- **[LBR-Stack](https://github.com/lbr-stack)** - KUKA LBR robot descriptions and control
- **[dvrk_urdf](https://github.com/shashank3199/dvrk_urdf)** - da Vinci robot descriptions
- **[moveit_config](https://github.com/suphasitpp/moveit_config)** - MoveIt configuration for motion planning
- **MoveIt 2** - For motion planning and kinematic solving capabilities
- **Standard ROS 2 packages:** robot_state_publisher, joint_state_publisher_gui, rviz2, interactive_markers

---

## Workspace Setup Example

```bash
# Create workspace
mkdir -p ~/my_ros2_ws/src && cd ~/my_ros2_ws

# Set up LBR stack with proper dependencies (includes fri and lbr_fri_idl)
source /opt/ros/humble/setup.bash
export FRI_CLIENT_VERSION=1.15
vcs import src --input https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/humble/lbr_fri_ros2_stack/repos-fri-${FRI_CLIENT_VERSION}.yaml

# Clone additional dependencies
cd src
git clone https://github.com/shashank3199/dvrk_urdf.git
git clone https://github.com/suphasitpp/da_vinci_tool_integration.git
git clone https://github.com/suphasitpp/moveit_config.git

# Build the workspace
cd ..
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Usage Sequence

### 1. Basic Setup (No RCM)
```bash
# Launch MoveIt + IK Solver
ros2 launch da_vinci_tool_integration launchpad_no_rcm.launch.py
```

### 2. Interactive Marker Only
```bash
# Launch interactive target marker
ros2 launch da_vinci_tool_integration interactive_marker_only.launch.py
```

### 3. RCM Command Center
```bash
# Launch RCM manager + GUI + PS5 teleop
ros2 launch da_vinci_tool_integration rcm_command_center.launch.py
```

### 4. Surgical Workspace Test
```bash
# Launch static surgical box visualization
ros2 run da_vinci_tool_integration static_surgical_box.py
```

### Launch Files

- **`launchpad_no_rcm.launch.py`** - MoveIt + IK Solver (basic setup)
- **`interactive_marker_only.launch.py`** - Interactive target marker only
- **`rcm_command_center.launch.py`** - RCM manager + GUI + PS5 teleop
- **`med7_combined_simple.launch.py`** - Basic robot visualization

---

## Package Architecture

This package works as part of a two-package system:

### This Package (`da_vinci_tool_integration`)
- **Robot Description:** URDF files for LBR Med7 + da Vinci PSM + custom adaptor
- **Kinematic Tools:** Forward/inverse kinematics solvers and interactive markers
- **Surgical Visualization:** Static surgical box with reference markers
- **RCM Management:** Remote Center of Motion constraint handling
- **Launch Files:** Complete setup sequences for different use cases

### Companion Package (`moveit_config`)
- **Motion Planning:** MoveIt configuration for path planning and execution
- **Joint Limits:** Velocity and acceleration constraints
- **Planning Algorithms:** Optimized for surgical robotics applications

### Key Python Modules
- **`ik_solver.py`** - Inverse kinematics solver
- **`interactive_target_marker.py`** - Interactive 6DOF marker
- **`static_surgical_box.py`** - Surgical workspace visualization
- **`rcm_manager.py`** - RCM constraint handling
- **`gui_pose_publisher.py`** - GUI pose control
- **`ps5_teleop_rcm.py`** - PS5 controller teleoperation

### URDF & Configuration Files (This Package)
- **`urdf/`** - Robot descriptions including LBR Med7, PSM tool, and custom adaptor
- **`launch/`** - Launch files for robot visualization and control
- **`rviz/`** - Pre-configured RViz settings optimized for surgical robotics

---

## Notes

- **Package Integration:** This package provides robot description and kinematic tools, while the companion `moveit_config` package provides motion planning configuration.
- The adaptor URDF and mesh files are included in `urdf/adaptor/` within this package.
- You do **not** need to clone or build any separate adaptor package.

- **Surgical Workspace:** Static surgical box provides configurable workspace with reference markers.
- **RCM Constraints:** Remote Center of Motion constraints maintain proper surgical entry point alignment.
- **Launch Sequence:** Use the launch files in sequence for different surgical robotics scenarios.
- If you encounter build errors related to missing FRI client, ensure you're using the proper setup method above.

---

## License

[MIT License](LICENSE) (or your chosen license)

---

## Contact

For questions or contributions, open an issue or pull request on [GitHub](https://github.com/suphasitpp/da_vinci_tool_integration). 
