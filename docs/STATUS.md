# STATUS

Update this file at the end of every phase. It is the handoff between sessions.

## Phase tracker

| Phase | State | Evidence or notes |
|---|---|---|
| 0 Environment and rules | done | Toolchain verified: Ubuntu 22.04.5, ROS 2 Humble, Gazebo 11.10.2, numpy 1.21.5, cv2 4.5.4. GPU: RTX 3050. Smoke test passed (/spawn_entity service found). git not installed yet (NEEDS-HUMAN). |
| 1 Package, world, URDFs | done | Build OK. All topics present. UAV odom (-10.0,-10.0,12.0), UGV odom (-12.015,-12.008,0.15) both within 0.05m. Camera ~10 Hz, scan ~10 Hz. UAV holds Z=12. UGV drives forward and stays upright. Camera frames arrive headless. Zero comments in project files. |
| 2 Patrol and mapper | not started | |
| 3 Planner | not started | |
| 4 Controller and launch | not started | |
| 5 Metrics and polish | not started | |
| 6 Hardening and submission | not started | |

## Decision log

| ID | Decision | State |
|---|---|---|
| D1 | World-frame odometry for both robots | confirmed: planar_move reports world coords at spawn, diff_drive with odometry_source=1 reports world coords. No fallback needed. |
| D2 | Arena geometry and placement | adopted, verified in world file |
| D3 | Two cost layers: /shared_costmap and /lidar_costmap | proposed |
| D4 | Unknown cells cost 30; planner waits for /uav/patrol_done | proposed |
| D5 | Self-contained world, HSV-safe materials | adopted, world has inline ground plane, sun, all materials. Gazebo material warnings about URDF ambient are cosmetic (headless rendering). |
| D6 | UAV yaw fixed at 0, coverage check | proposed |
| D7 | UGV physical design | adopted, URDF uses fixed caster joints (URDF does not support ball type) |
| D8 | Planner details | proposed |
| D9 | Terrain-aware regulated pure pursuit | adopted (Nav2 speed filter and RPP approach) |
| D10 | Launch contract | adopted, gzserver loaded with -s flags for system plugins, gzclient conditional on gui arg |
| D11 | Dependencies | proposed |
| D12 | Mission report metrics | proposed |
| D13 | Debug segmentation image | proposed |
| D14 | Verification tools | proposed |

## Deviations from the original PRD, ARCHITECTURE, and PHASES files

| Original | Change | Reason |
|---|---|---|
| Mode A treats cells below 95 as 30 on /shared_costmap | Baseline reads /lidar_costmap | Mines are 100 in the shared grid, so the baseline would avoid them |
| Constant 0.45 m/s controller | Terrain speed limit, curvature regulation, smoother (D9) | Otherwise the road route (46 m) loses to the diagonal (34 m) on time |
| Grid initialised to 30 | Initialised to -1, read as 30 by planner | Shows the UAV reveal in RViz, matches the locked unknown value |
| Planned path at 1 Hz | 2 Hz | Faster reaction to newly seen LiDAR obstacles |
| setup.py and launch in Phase 5 | Minimal versions in Phase 1, finalised in Phase 4 and 5 | Robots cannot be tested without them |
| Patrol called a lawnmower pattern | Called a waypoint patrol | The waypoint list is not a lawnmower |
| Minefield and mud both on the diagonal | Mud is off the diagonal | Mud is at (4,-5) |
| World file has plugin tags for init and factory | Plugins loaded via gzserver -s flags | They are system plugins, not world plugins. World-level tags cause type mismatch error. |
| URDF uses ball joints for casters | Changed to fixed joints | URDF parser does not support ball joint type |

## Last session notes

Phase 1 complete. All verification checks pass. git needs to be installed before commits can be made. Items for human eye check: red patch rendering at ~139x139 px, patch orientation, flat shading.
