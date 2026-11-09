#!/usr/bin/env bash
export GAZEBO_MODEL_DATABASE_URI=""
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
pkill -9 -f "tactical_recon/lib/tactical_recon" 2>/dev/null || true
source /opt/ros/humble/setup.bash
source /home/fury/College/Robotics/uav_ugv_ws/install/setup.bash
ros2 launch tactical_recon sim_mission.launch.py cooperative_mode:=true "$@"
