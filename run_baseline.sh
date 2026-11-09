#!/usr/bin/env bash
export GAZEBO_MODEL_DATABASE_URI=""
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
pkill -9 -f "tactical_recon/lib/tactical_recon" 2>/dev/null || true
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${UAV_UGV_WS:-$SCRIPT_DIR}"
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
ros2 launch tactical_recon sim_mission.launch.py cooperative_mode:=false "$@"
