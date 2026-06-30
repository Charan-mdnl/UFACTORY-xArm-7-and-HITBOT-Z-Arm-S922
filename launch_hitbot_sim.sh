#!/bin/bash
# Launch only the HITBOT Z-Arm S922 in simulation (fake hardware + MoveIt)
echo "═══════════════════════════════════════════════════"
echo " HITBOT Z-Arm S922 — Simulation Mode"
echo "═══════════════════════════════════════════════════"

export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export ROS_DOMAIN_ID=42

killall -9 rviz2 move_group ros2_control_node 2>/dev/null
sleep 1

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
[ ! -f "$WORKSPACE_DIR/install/setup.bash" ] && echo "Error: Run ./setup.sh first" && exit 1

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# Use the hitbot_moveit_config's simulation launch with mock hardware
ros2 launch hitbot_moveit_config moveit_rviz.launch.py
