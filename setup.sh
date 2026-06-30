#!/bin/bash
# ═══════════════════════════════════════════════════════════════════
# setup.sh — Build the dual-arm workspace
#
# Usage:
#   ./setup.sh          ← full build
#   ./setup.sh clean    ← clean + rebuild
# ═══════════════════════════════════════════════════════════════════

set -e

echo "═══════════════════════════════════════════════════"
echo " Building Dual-Arm Workspace"
echo " xArm 7 + HITBOT Z-Arm S922"
echo "═══════════════════════════════════════════════════"

# Remove Anaconda/Miniforge from paths
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "miniforge\|anaconda\|conda\|miniconda" | tr '\n' ':' | sed 's/:$//')

WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ "$1" = "clean" ]; then
    echo "=> Cleaning build artifacts..."
    rm -rf "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/install" "$WORKSPACE_DIR/log"
fi

source /opt/ros/humble/setup.bash

echo "=> Building with colcon..."
cd "$WORKSPACE_DIR"
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    --parallel-workers $(nproc)

echo ""
echo "═══════════════════════════════════════════════════"
echo " Build complete!"
echo ""
echo " Quick start:"
echo "   ./launch_dual_arm.sh sim     # Both arms in simulation"
echo "   ./launch_dual_arm.sh real    # Both arms with hardware"
echo "   ./launch_xarm_sim.sh         # xArm 7 only (sim)"
echo "   ./launch_hitbot_sim.sh       # HITBOT only (sim)"
echo "═══════════════════════════════════════════════════"
