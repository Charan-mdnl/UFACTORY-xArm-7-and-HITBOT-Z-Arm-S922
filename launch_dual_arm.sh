#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# launch_dual_arm.sh — Launch both xArm 7 + HITBOT Z-Arm S922
#
# Usage:
#   ./launch_dual_arm.sh sim    ← simulation (mock hardware)
#   ./launch_dual_arm.sh real   ← real hardware
# ═══════════════════════════════════════════════════════════════════

MODE="${1:-sim}"

echo "═══════════════════════════════════════════════════"
echo " Dual-Arm: xArm 7 + HITBOT Z-Arm S922"
echo " Mode: $MODE"
echo "═══════════════════════════════════════════════════"

# Remove Anaconda/Miniforge from paths (conflicts with ROS 2)
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')

export ROS_DOMAIN_ID=42

# Fix RViz crash on Intel HD 4400 (Lenovo Z50-70)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Kill leftover processes
echo "=> Cleaning up stale processes..."
killall -9 gzserver gzclient rviz2 move_group ros2_control_node robot_state_publisher 2>/dev/null
sleep 1

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "Error: Workspace not built! Run: ./setup.sh"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

echo "=> ROS_DOMAIN_ID = $ROS_DOMAIN_ID"

if [ "$MODE" = "real" ]; then
    echo "=> Launching REAL hardware mode..."
    echo "   xArm 7 IP:  192.168.1.230"
    echo "   HITBOT IP:   192.168.58.2:8080"
    ros2 launch dual_arm_config dual_arm_real.launch.py
elif [ "$MODE" = "sim" ]; then
    echo "=> Launching SIMULATION mode (mock hardware)..."
    ros2 launch dual_arm_config dual_arm_sim.launch.py
else
    echo "Error: Unknown mode '$MODE'. Use 'sim' or 'real'."
    exit 1
fi
