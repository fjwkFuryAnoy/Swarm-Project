# PHASES: build plan with verification

Rule for every phase: a phase is done only when every "Done when" line has raw evidence (command output or a screenshot). Tag the commit `phase-N-done` before starting the next one.

## How we work

1. Claude writes the complete files for the phase.
2. You place them in the workspace, build, and run the verification steps.
3. You paste the raw outputs and any errors back. Claude fixes and re-issues full files, not patches.
4. Antigravity's agent is optional. Use it for build-and-debug loops. When you do, start each phase with this prompt:

```
Read AGENTS.md, docs/PRD.md, docs/ARCHITECTURE.md, docs/PHASES.md and docs/STATUS.md. Implement Phase N only, touching only the files listed for Phase N. Build, run every verification step, and show raw output. Update docs/STATUS.md and stop. If anything in ARCHITECTURE.md conflicts with what you observe, stop and report it.
```

## Schedule (10 days)

| Day | Work |
|---|---|
| 1 | Phase 0, start Phase 1 |
| 2 | Finish Phase 1 (plugin debugging is the usual time sink) |
| 3 to 4 | Phase 2 |
| 5 | Phase 3 |
| 6 | Phase 4 |
| 7 | Phase 5, feature freeze at end of day |
| 8 | Phase 6 hardening and repeat runs |
| 9 to 10 | Report, video, buffer. No new features. |

If Phase 1 slips past day 2, drop D12 (metrics) first, then RViz polish.

---

## Phase 0: Environment and rules

Files: AGENTS.md and docs/*.md in ~/uav_ugv_ws. Delete the old PRD.md, PHASES.md, ARCHITECTURE.md, and AGENT_RULES.md first so the agent never reads conflicting versions. No code.

Steps:
1. Copy AGENTS.md to ~/uav_ugv_ws/AGENTS.md and the four docs (PRD, ARCHITECTURE, PHASES, STATUS) to ~/uav_ugv_ws/docs/. Open ~/uav_ugv_ws as the Antigravity workspace.
2. Ask the agent: "List the hard rules you were given." It must mention no comments, rclpy only, Gazebo Classic only. If it does not, the rules file was not loaded.
3. Check the toolchain:

```
lsb_release -d
printenv ROS_DISTRO
gazebo --version
python3 -c "import numpy, cv2, cv_bridge; print(numpy.__version__, cv2.__version__)"
dpkg -l | grep -E "ros-humble-gazebo-ros-pkgs|ros-humble-cv-bridge|ros-humble-robot-state-publisher|ros-humble-rviz2|ros-humble-image-view|python3-opencv"
```

4. Install anything missing:

```
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-image-view ros-humble-rqt-image-view python3-opencv
```

5. If the numpy line prints 2.x or cv_bridge fails to import: `pip3 install --user "numpy<2"`.
6. Check which GPU Gazebo uses:

```
nvidia-smi
glxinfo | grep "OpenGL renderer"
```

If the renderer is not the NVIDIA card, launch with `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`.
7. Smoke test: `ros2 launch gazebo_ros gazebo.launch.py`, then in another terminal `ros2 service list | grep spawn_entity`. If Gazebo hangs at startup, run `export GAZEBO_MODEL_DATABASE_URI=""` and retry.
8. `cd ~/uav_ugv_ws && git init && git add -A && git commit -m "phase 0"`.

Done when: rules confirmed by the agent, /spawn_entity service exists, numpy below 2, git initialised.

---

## Phase 1: Package plumbing, world, two URDFs

Files: package.xml, setup.py, setup.cfg, resource/tactical_recon, tactical_recon/__init__.py, launch/sim_mission.launch.py (minimal), worlds/military_sector.world, urdf/scout_uav.urdf, urdf/recon_ugv.urdf.

The minimal launch file is plumbing needed to test the robots. It is the one file pulled forward from the original manifest order.

Steps:
1. Build the package with all data_files and four console_scripts registered (node modules come later).
2. Write the world per ARCHITECTURE section 8, D2, D5. Plugins: libgazebo_ros_init.so, libgazebo_ros_factory.so.
3. Write scout_uav.urdf: planar_move, camera pitched down, gravity 0, D1 world odometry.
4. Write recon_ugv.urdf: diff_drive, ray sensor, D7 dimensions, D1 world odometry.
5. Write the minimal launch: gazebo, two robot_state_publishers, two spawns, static map to odom, args per D10.
6. Build and launch.

Verify:

```
ros2 topic list | grep -E "uav|ugv|clock"
ros2 topic hz /uav/camera/image_raw
ros2 topic hz /ugv/scan
ros2 topic echo /uav/odom --once --field pose.pose.position
ros2 topic echo /ugv/odom --once --field pose.pose.position
ros2 run rqt_image_view rqt_image_view
```

Motion tests:

```
ros2 topic pub -r 10 /ugv/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
ros2 topic pub -r 10 /uav/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0}}"
```

Camera calibration test: relaunch with `uav_x:=0.0 uav_y:=0.0`. The image must show the red 3 m patch centered, about 139 x 139 px, with dirt around it. Then check orientation: the green and mud patches must be on the sides predicted by "image up = world +X, image left = world +Y".

Done when:
- Both odom topics read the spawn pose within 0.05 m at t = 0 (D1). If not, record the fallback in STATUS.md.
- /uav/camera/image_raw about 10 Hz, /ugv/scan about 10 Hz.
- The UAV holds Z = 12 and does not fall; the UGV moves forward and stays upright.
- Red patch renders about 139 x 139 px at (0, 0) and colors look flat (no shading gradients).
- UGV drives over the patches without being blocked.

Traps: package builds but patches block the rover (a collision element slipped in); camera image is black (GPU or GL context); UAV drifts down (gravity set on the wrong element); odom starts at (0, 0) (D1).

---

## Phase 2: UAV patrol and terrain mapper

Files: uav_patrol_node.py, terrain_mapper_node.py, rviz/mission_view.rviz, tools/check_costmap.py.

The planned path and goal marker are added to RViz in Phase 5.

Steps:
1. uav_patrol_node: proportional waypoint follower per ARCHITECTURE section 8 and D6, publishes /uav/patrol_done.
2. terrain_mapper_node: camera layer (vectorised numpy over the step-8 pixel grid, HSV bands from section 6), LiDAR layer, fusion and publishing per D3, and /uav/debug_segmentation per D13. Costmap geometry from section 4.
3. rviz/mission_view.rviz: two OccupancyGrid displays (/shared_costmap, /lidar_costmap) with fixed frame map, LaserScan, TF, and an Image display on /uav/debug_segmentation.
4. tools/check_costmap.py: subscribes once to both costmaps and prints counts per value.

Verify (rover stationary, patrol finished):

```
ros2 run tactical_recon uav_patrol_node
ros2 run tactical_recon terrain_mapper_node
python3 ~/uav_ugv_ws/tools/check_costmap.py
```

Expected counts from /shared_costmap, with LiDAR cells subtracted using /lidar_costmap:

| Class | Expected cells | Tolerance |
|---|---|---|
| 100 (mines, inflated) | about 722 | 650 to 800 |
| 80 (mud) | about 400 | 360 to 440 |
| 5 (road) | about 2354 | 2150 to 2550 |

Then drive the rover in a circle with `ros2 topic pub` and watch /lidar_costmap fill with the walls and blocks in RViz.

Done when:
- Counts are inside tolerance, so every hazard and road cell is covered (D6 coverage check). If the road count is short, apply the D6 fallback waypoint.
- Mines, mud and road appear in RViz at the right places, aligned with the Gazebo world.
- /uav/patrol_done is True after the last waypoint.
- /uav/debug_segmentation shows the classified blocks and the HUD text in RViz.
- Mapper CPU stays below one core at 10 Hz camera input.

Traps: red and blue swapped (BGR versus RGB); map mirrored in X or Y (sign error in the projection); blocks overwriting road cells (violates the D3 write rules); QoS mismatch so RViz shows nothing (costmaps are transient_local).

---

## Phase 3: Global planner

Files: global_planner_node.py, tools/check_path.py.

Steps:
1. Weighted A* per ARCHITECTURE section 7 and D8, with parameter cooperative_mode and the D4 start gate.
2. Publish /ugv/planned_path at 2 Hz in frame map.
3. tools/check_path.py: subscribes once to the path and to /shared_costmap and prints path length, pose count, the number of poses on cells >= 95, and the percentage on road cells.

Verify, with the simulator, mapper, and patrol running and the rover stationary:

```
ros2 run tactical_recon global_planner_node --ros-args -p cooperative_mode:=false -p use_sim_time:=true
python3 ~/uav_ugv_ws/tools/check_path.py
ros2 run tactical_recon global_planner_node --ros-args -p cooperative_mode:=true -p use_sim_time:=true
python3 ~/uav_ugv_ws/tools/check_path.py
```

| Mode | Path length | Poses on cost >= 95 | Poses on road |
|---|---|---|---|
| false | about 34 to 37 m | greater than 0 | near 0% |
| true | about 46 to 49 m | 0 | at least 80% |

Done when: both rows hold, the path appears in RViz, and no path is published before /uav/patrol_done is True. Also confirm the planner replans around a block after the LiDAR first sees it (baseline diagonal bends around (-8,-8) or (4,4)).

Traps: planner treats unknown cells as impassable; start cell trapped by inflation; corner cutting between diagonal obstacles; planner blocking the executor (run A* inside the timer callback and keep it under 50 ms, or use a cell-count cap).

---

## Phase 4: Controller and full launch

Files: ugv_controller_node.py, launch/sim_mission.launch.py (final version).

Steps:
1. ugv_controller_node per ARCHITECTURE section 7 and D9 (terrain-aware regulated pure pursuit: terrain speed limit, curvature regulation, rotate-to-heading, velocity smoother).
2. Final launch: everything from D10, cooperative_mode argument forwarded to both the planner and the controller.
3. Run both modes end to end.

Verify:

```
ros2 launch tactical_recon sim_mission.launch.py cooperative_mode:=false
ros2 launch tactical_recon sim_mission.launch.py cooperative_mode:=true
```

Done when:
- Baseline: the UGV drives the diagonal, crosses the red patches, reaches the goal in about 65 to 75 s.
- Cooperative: the UGV follows the green corridor, never enters a red cell, reaches the goal in about 35 to 45 s.
- Cooperative time is clearly below baseline time. If it is not, the speed governor (D9) is wrong or disabled.
- The UGV stops within 0.5 m of (12, 12) and stays stopped.

Traps: UGV overshoots the corridor corner at (-12, 10) (check the curvature factor and the rotate-to-heading threshold); wheel slip on road (lower the acceleration limit or lower v0 and record it); controller reading stale path indices after a replan.

---

## Phase 5: Metrics, RViz polish, demo scripts

Files: ugv_controller_node.py (D12 metrics), rviz/mission_view.rviz (final), plus optional scripts run_baseline.sh and run_cooperative.sh in the workspace root.

Steps:
1. Add the D12 mission report line to the controller.
2. RViz: add the planned path and a goal marker at (12, 12) to the existing displays, with UGV and UAV TF frames and distinct costmap palettes.
3. Bash scripts that kill stale simulators, source the workspace, and launch one mode.

Done when: each mode prints one report line with elapsed_s, distance_m, mine_time_s, mud_time_s, road_time_s. Expected: baseline mine_time_s greater than 0, cooperative mine_time_s equal to 0.

---

## Phase 6: Hardening and submission

Steps:
1. Clean rebuild: `rm -rf build install log && colcon build --symlink-install`, then run both modes.
2. Run each mode 3 times. Record the report lines in a table. Results must be consistent.
3. Grep the whole project for comments to confirm rule 1:

```
grep -rnE "^\s*#|<!--|\"\"\"" ~/uav_ugv_ws/src/tactical_recon --include="*.py" --include="*.xml" --include="*.urdf" --include="*.world" --include="*.sh"
```

Only an acceptable shebang line may appear.
4. Record the demo video: baseline then cooperative, split screen with Gazebo and RViz.
5. Ask Claude for the README and the report outline using the recorded numbers.

Done when: three consistent runs per mode, a clean-build run, a clean grep, and the video recorded.

---

## Troubleshooting table

| Symptom | Likely cause |
|---|---|
| Gazebo hangs on startup | Stale gzserver, or model database fetch. `pkill -9 gzserver`; `export GAZEBO_MODEL_DATABASE_URI=""` |
| Camera topic exists but image is black or empty | No GL context or wrong GPU. Check PRIME offload; as a last resort `LIBGL_ALWAYS_SOFTWARE=1` |
| All hazards appear mirrored | Sign error in the section 5 projection or wrong camera orientation |
| Colors misclassified | Shading in materials, or RGB and BGR mixed up (D3, D5) |
| cv_bridge import error mentioning numpy | numpy 2.x installed. `pip3 install --user "numpy<2"` |
| Node runs but nothing happens | use_sim_time not set, or /clock missing (check libgazebo_ros_init.so) |
| Costmap invisible in RViz | QoS: set durability to transient local in the display |
| `ros2 run` cannot find executable | setup.py console_scripts missing, or built without sourcing install/setup.bash |
