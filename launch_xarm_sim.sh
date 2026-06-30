#!/bin/bash
# Launch only the xArm 7 in simulation (fake hardware + MoveIt)
echo "═══════════════════════════════════════════════════"
echo " xArm 7 — Simulation Mode"
echo "═══════════════════════════════════════════════════"

export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export ROS_DOMAIN_ID=42

killall -9 gzserver gzclient rviz2 move_group 2>/dev/null
sleep 1

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
[ ! -f "$WORKSPACE_DIR/install/setup.bash" ] && echo "Error: Run ./setup.sh first" && exit 1

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

ros2 launch xarm_planner xarm7_planner_fake.launch.py
