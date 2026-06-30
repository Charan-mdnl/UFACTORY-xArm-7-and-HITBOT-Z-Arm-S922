# UFACTORY xArm 7 & HITBOT Z-Arm S922 Unified ROS 2 Workspace

A unified, self-contained ROS 2 Humble workspace for controlling both the **UFACTORY xArm 7** (7 DoF) and the **HITBOT Z-Arm S922** (6 DoF) robotic arms simultaneously in simulation (using mock hardware) and real-world hardware.

## 🚀 Features
- **Dual-Arm Control**: Control both arms independently or simultaneously from a single MoveIt 2 instance.
- **Upright SCARA Mount**: Fixed visual/collision orientation of the HITBOT Z-Arm (rotated 90° on the X-axis) so that it stands vertically next to the xArm 7 instead of lying flat.
- **Domain Isolation**: Uses `ROS_DOMAIN_ID=42` to isolate your workspace and prevent conflicts with other robots on the network.
- **Conda Conflict Free**: Automatically strips Anaconda/Miniforge libraries from active paths to prevent conflicts with ROS 2 and Gazebo.
- **Clean Configuration**: Removed fixed links from active planning groups and formatted OMPL request adapters correctly as a space-separated string to ensure crash-free execution.

---

## 📂 Repository Structure
```
.
├── setup.sh                 # Cleans and builds the workspace
├── launch_dual_arm.sh       # Usage: ./launch_dual_arm.sh [sim|real]
├── launch_xarm_sim.sh       # xArm 7 simulation only
├── launch_xarm_real.sh      # xArm 7 real hardware (IP: 192.168.1.230)
├── launch_hitbot_sim.sh     # HITBOT simulation only
├── launch_hitbot_real.sh    # HITBOT real hardware (IP: 192.168.58.2)
├── RUN_COMMANDS.txt         # Quick start cheat sheet
└── src/
    ├── xarm_ros2/           # xArm 7 ROS 2 core packages
    ├── hitbot_description/  # HITBOT URDF meshes and geometry
    ├── hitbot_moveit_config/ # HITBOT MoveIt configs
    ├── hitbot_hardware/     # HITBOT TCP driver and hardware interface
    ├── hitbot_imitation/    # HITBOT imitation learning package
    ├── hitbot-api/          # Standalone python API wrapper
    ├── cwsfa_hitbot_api/    # Alternative Python API wrapper
    └── dual_arm_config/     # Unified MoveIt 2 integration package
```

---

## 🛠️ Installation & Setup

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- MoveIt 2 for ROS 2 Humble

### 1. Build the Workspace
Clone and run the compilation script:
```bash
git clone https://github.com/Charan-mdnl/UFACTORY-xArm-7-and-HITBOT-Z-Arm-S922.git ~/dual_arm_ws
cd ~/dual_arm_ws
./setup.sh clean
```

---

## 💻 Running the Simulation
To run both arms in simulation (fake hardware / mock_components):
```bash
./launch_dual_arm.sh sim
```
This will launch:
1. `ros2_control_node` with simulated hardware interfaces.
2. `robot_state_publisher` publishing the integrated URDF.
3. MoveIt 2 `move_group` server.
4. RViz with the interactive markers loaded.

---

## 🤖 Controlling Real Hardware

### 1. Subnet Setup
Both robots operate on different subnets. Your control PC must be connected to both. You can assign two IP addresses to the same Ethernet interface (e.g., `enp1s0`):
```bash
# Add IP for xArm 7 subnet (default IP: 192.168.1.230)
sudo ip addr add 192.168.1.100/24 dev enp1s0

# Add IP for HITBOT subnet (default IP: 192.168.58.2)
sudo ip addr add 192.168.58.100/24 dev enp1s0

# Verify connections
ping 192.168.1.230
ping 192.168.58.2
```

### 2. Launch the Hardware Nodes
To connect and control the real hardware controllers:
```bash
./launch_dual_arm.sh real
```

---

## 🎮 Moving the Robots in RViz
1. In the bottom-left **MotionPlanning** panel of RViz, click the **Planning** tab.
2. Under **Planning Group**:
   - Select `xarm7` to control the xArm 7 (DoF: 7).
   - Select `hitbot_arm` to control the HITBOT Z-Arm S922 (DoF: 6).
3. Drag the interactive marker ball at the tool tip to set the target pose.
4. Click **Plan** to visualize, then **Execute** to move.

---

## 🛠️ Troubleshooting (RViz Crashes / OpenGL Errors)

If RViz2 crashes or segmentation faults (`exit code -11` / `SIGSEGV`) on startup, it is usually due to OpenGL driver conflicts with the rendering engine (especially common on integrated graphics like Intel HD 4000 series or NVIDIA/AMD hybrid setups).

To prevent these graphics crashes, all launch scripts automatically override Mesa and enforce software rendering:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
```

If you encounter issues running the scripts, make sure to execute them directly in your **desktop user terminal** (instead of a remote SSH or headless session), as RViz requires a valid desktop display context (`$DISPLAY`).

