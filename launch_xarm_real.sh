#!/bin/bash
# Launch only the xArm 7 with REAL hardware
echo "═══════════════════════════════════════════════════"
echo " xArm 7 — Real Hardware Mode"
echo "═══════════════════════════════════════════════════"

export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export ROS_DOMAIN_ID=42

# Fix RViz crash on Intel HD 4400 (Lenovo Z50-70)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

killall -9 rviz2 move_group 2>/dev/null
sleep 1

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
[ ! -f "$WORKSPACE_DIR/install/setup.bash" ] && echo "Error: Run ./setup.sh first" && exit 1

source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

echo "=> Connecting to xArm 7 at 192.168.1.230..."
ros2 launch xarm_planner xarm7_planner_realmove.launch.py robot_ip:=192.168.1.230 add_gripper:=false
