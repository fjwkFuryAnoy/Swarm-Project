# ARCHITECTURE: locked specification and active design decisions

Sections 1 to 8 are locked. Section 9 closes gaps found while reviewing the original PRD, ARCHITECTURE, and PHASES files. Section 9 is active unless docs/STATUS.md records a change.

## 1. File manifest

```
~/uav_ugv_ws/
├── AGENTS.md
├── docs/ (PRD.md, ARCHITECTURE.md, PHASES.md, STATUS.md)
├── tools/ (check_costmap.py, check_path.py)
└── src/tactical_recon/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/tactical_recon
    ├── launch/sim_mission.launch.py
    ├── worlds/military_sector.world
    ├── urdf/scout_uav.urdf
    ├── urdf/recon_ugv.urdf
    ├── rviz/mission_view.rviz
    └── tactical_recon/
        ├── __init__.py
        ├── uav_patrol_node.py
        ├── terrain_mapper_node.py
        ├── global_planner_node.py
        └── ugv_controller_node.py
```

## 2. Frames and TF

- Global frame map, odometry frame odom. Launch file publishes a static identity transform map to odom.
- libgazebo_ros_planar_move publishes odom to uav_base_link and /uav/odom.
- libgazebo_ros_diff_drive publishes odom to ugv_base_link and /ugv/odom.

## 3. Topic contracts

| Topic | Type | Publisher | Rate | Subscribers |
|---|---|---|---|---|
| /uav/cmd_vel | geometry_msgs/Twist | uav_patrol_node | 10 Hz | planar_move plugin |
| /uav/odom | nav_msgs/Odometry | planar_move plugin | 20 Hz | patrol, mapper |
| /uav/camera/image_raw | sensor_msgs/Image, 640x480 RGB8 | camera plugin | 10 Hz | mapper |
| /uav/patrol_done | std_msgs/Bool, transient_local | uav_patrol_node | once | planner |
| /uav/debug_segmentation | sensor_msgs/Image | terrain_mapper_node | 5 Hz | RViz |
| /ugv/scan | sensor_msgs/LaserScan | ray sensor plugin | 10 Hz | mapper |
| /ugv/odom | nav_msgs/Odometry | diff_drive plugin | 20 Hz | mapper, planner, controller |
| /shared_costmap | nav_msgs/OccupancyGrid, transient_local | terrain_mapper_node | 2 Hz | planner, controller, RViz |
| /lidar_costmap | nav_msgs/OccupancyGrid, transient_local | terrain_mapper_node | 2 Hz | planner, controller, RViz |
| /ugv/planned_path | nav_msgs/Path | global_planner_node | 2 Hz | controller, RViz |
| /ugv/cmd_vel | geometry_msgs/Twist | ugv_controller_node | 20 Hz | diff_drive plugin |

## 4. OccupancyGrid

- frame_id map, 150 x 150 cells, 0.20 m/cell, origin (-15.0, -15.0, 0.0).
- World to grid: gx = int((Xw + 15.0) / 0.20), gy = int((Yw + 15.0) / 0.20), valid for 0 <= gx, gy < 150, index = gy * 150 + gx.
- Grid to world center: Xw = -15.0 + (gx + 0.5) * 0.20, Yw = -15.0 + (gy + 0.5) * 0.20.
- Goal (12, 12) is cell (135, 135).
- data is signed int8 in [-1, 100]. Never assign a value above 100. Allowed values: unknown -1, green road 5, dirt 30, mud 80, mine or LiDAR wall 100 (with 2-cell inflation).

## 5. UAV pinhole projection

- Z_uav = 12.0 m. Camera joint rpy="0 1.570796 0". 640 x 480, hfov 1.047198 rad, fx = fy = 554.256, cx = 320, cy = 240. Iterate pixels in steps of 8.
- dx_body = -(v - cy) * (Z_uav / fy), dy_body = -(u - cx) * (Z_uav / fx)
- Xw = X_uav + dx_body * cos(theta) - dy_body * sin(theta)
- Yw = Y_uav + dx_body * sin(theta) + dy_body * cos(theta)
- theta comes from the /uav/odom quaternion.

## 6. HSV classification

Convert with the code matching the cv_bridge encoding (rgb8 with COLOR_RGB2HSV, bgr8 with COLOR_BGR2HSV).

- Red (100): H in [0,10] or [170,180], S in [100,255], V in [80,255]
- Yellow (80): H in [18,34], S in [100,255], V in [80,255]
- Green (5): H in [35,85], S in [80,255], V in [50,255]

## 7. Planner and controller (locked parts)

- A* is 8-connected from the UGV cell to the goal cell. Cells with cost >= 95 are impassable.
- step_cost = euclid(A, B) * (1.0 + cost[B] / 10.0). Road is 1.5x, dirt 4.0x, mud 9.0x.
- Pure pursuit, lookahead 0.8 m, stop within 0.5 m of (12, 12).
- cooperative_mode (default True) is a boolean parameter on global_planner_node.

## 8. Arena, spawns, plugins, patrol

- Arena 30 x 30 m, X and Y in [-15, 15], 2.0 m walls, 4 concrete blocks 1.5 x 1.5 x 2.0.
- Terrain boxes 0.02 m thick at Z = 0.01, no collision. Red (0.9, 0.0, 0.0) at (0,0) and (-4,-4). Yellow (0.9, 0.75, 0.0) at (4,-5). Green (0.0, 0.8, 0.0) L-corridor (-12,-12) to (-12,10) to (12,12).
- UGV spawn (-12, -12, 0.2, yaw 0). UAV spawn (-10, -10, 12, yaw 0), gravity 0. Goal (12, 12).
- World loads libgazebo_ros_init.so and libgazebo_ros_factory.so.
- UAV: libgazebo_ros_planar_move.so (namespace /uav) and libgazebo_ros_camera.so. UGV: libgazebo_ros_diff_drive.so (namespace /ugv) and libgazebo_ros_ray_sensor.so with output_type sensor_msgs/LaserScan, 360 deg, 10 Hz.
- UAV waypoints at 2.0 m/s: (-10,-10), (0,0), (-10,8), (8,8), (4,-4), (10,10).

---

## 9. Active design decisions

### D1. Odometry in the world frame

/uav/odom and /ugv/odom must report world coordinates. At t = 0 they must read (-10,-10) and (-12,-12) within 0.05 m. Configure both plugins for world odometry source. Fallback if a plugin starts at zero: nodes take parameters uav_spawn_x, uav_spawn_y, ugv_spawn_x, ugv_spawn_y and add them. Decide by test in Phase 1.

### D2. Arena geometry

- Walls: four boxes 2.0 m tall, 0.2 m thick, collision on, inner faces at exactly +/-15.0.
- Concrete blocks, collision on, centers (-8,-8), (4,4), (8,-2), (-4,8).
- Red 3.0 x 3.0 at (0,0) and (-4,-4). Yellow 4.0 x 4.0 at (4,-5). Axis-aligned, center Z 0.01, no collision element.
- Green road, width 2.0 m, four boxes: vertical center (-12,-1) size 2.0 x 22.0; corner joint center (-12,10) size 2.0 x 2.0; slanted box from (-11,10.0833) to (13,12.0833), center (1,11.0833), length 24.083, yaw 0.0831 rad.
- No goal marker in the world. Show the goal in RViz only.
- The yellow mud is not on the diagonal. It only penalises off-road detours in cooperative mode.

### D3. Two cost layers

The original rule "if cooperative_mode is False treat cells below 95 as cost 30" still leaves mines at 100 in the grid, so the baseline planner would avoid them. A single grid cannot separate a mine from a wall. terrain_mapper_node therefore keeps two layers and publishes two topics, both reliable with transient_local depth 1:

- /shared_costmap: fused. Value 100 where the LiDAR layer is set, otherwise the terrain value. Initial value -1, so RViz shows the map being revealed as the UAV flies.
- /lidar_costmap: LiDAR only. Every cell 30, except LiDAR obstacles and their inflation at 100.

Planner and controller read /shared_costmap in cooperative mode and /lidar_costmap in baseline mode.

Terrain write rules: road (5), mud (80), and mine (100) overwrite any value. Pixels matching no class write 30 only to cells still at -1; they never downgrade a hazard or road. Red cells are dilated by 2 cells (5 x 5 square) when published. LiDAR endpoints are dilated by 2 cells and are persistent; skip endpoints outside the grid.

### D4. Unknown cells and start gate

- Planner and controller read -1 as 30.
- uav_patrol_node publishes /uav/patrol_done (True, transient_local) at the last waypoint. The planner publishes no path until it receives True, in both modes (parameter wait_for_patrol, default true), so both modes start from the same state.

### D5. Materials and world

- Self-contained world: ground plane and sun defined inline, no model:// includes.
- Ground dirt RGB (0.50, 0.45, 0.38), saturation about 61, below every band's S minimum. Concrete grey (0.5, 0.5, 0.5).
- Patches: ambient = diffuse = spec RGB, specular 0, emissive 0. White directional light with high ambient.
- UGV body blue (0.15, 0.25, 0.6), wheels near-black. UAV body light grey, mounted above the camera so it stays out of the image.
- No scene object may use a hue inside the red, yellow, or green bands at S >= 80.

### D6. UAV yaw and coverage

- Yaw stays 0. The patrol node commands only linear x and y, proportional controller capped at 2.0 m/s, waypoint tolerance 0.3 m.
- Image footprint at Z = 12: 10.39 m along X by 13.86 m along Y.
- Phase 2 coverage check: counts of road, mine, and mud cells must match the expected values. The left edge of the road near y in (-0.87, -0.69) is marginal. If the check fails, insert waypoint (-10, 0) between (0,0) and (-10,8) and record it in STATUS.md.

### D7. UGV design

- Chassis 0.6 x 0.4 x 0.15 m, about 10 kg. Two drive wheels (radius 0.1 m, separation 0.5 m) and two frictionless caster spheres. Lidar on top, 360 samples, range 0.12 to 10.0 m.
- Link names: uav_base_link, uav_camera_link, ugv_base_link, ugv_laser_link, ugv_left_wheel, ugv_right_wheel. Robots: scout_uav, recon_ugv.

### D8. Planner

- Replan at 2 Hz from the current UGV pose.
- Disallow diagonal moves that cut a corner between impassable cells.
- Treat cells within 3 cells of the start as passable.
- No path: publish an empty path and warn once per second.
- Path frame_id map, poses at cell centers.

### D9. Controller: terrain-aware regulated pure pursuit

A constant-speed controller makes the cooperative route slower: road route about 46 m against about 34 m for the diagonal. At the original 0.45 m/s that is about 102 s against about 76 s, so the demo would show the cooperative rover losing on time. Real systems avoid this with a speed limit derived from the costmap (the Nav2 costmap speed filter) combined with curvature regulation (Nav2 Regulated Pure Pursuit) and a velocity smoother. The controller does the same:

1. Terrain speed limit: v_terrain = v0 / (1 + c / 10) with v0 = 2.0 m/s. This is the same cost model the planner minimises, so driving time is proportional to path cost. c is the maximum costmap value over the path poses from the current pose out to 1.5 m ahead (so the rover slows before mud), with -1 read as 30 and values capped at 90. Result: about 1.33 m/s on road, 0.5 m/s on dirt, 0.22 m/s in mud. Lower bound 0.15 m/s.
2. Curvature regulation: linear speed multiplied by max(0.25, cos(alpha)).
3. Rotate to heading: if |alpha| > 0.6 rad, linear.x = 0 and angular.z = 0.8 * sign(alpha).
4. Steering: omega = 2 * v * sin(alpha) / L with L = 0.8 m, clamped to +/-1.5 rad/s.
5. Velocity smoother: limit acceleration to 1.0 m/s^2 and deceleration to 2.0 m/s^2 at 20 Hz, to prevent wheel slip.
6. Costmap source: /shared_costmap in cooperative mode, /lidar_costmap in baseline mode, so the baseline rover is terrain-blind and drives at dirt speed. ugv_controller_node therefore also has the cooperative_mode parameter.
7. Stop within 0.5 m of (12, 12). Publish zero velocity when no path is available.

Expected times: cooperative about 35 to 45 s, baseline about 65 to 75 s.

### D10. Launch contract

- Arguments: cooperative_mode (default true), gui (default true), uav_x and uav_y (defaults -10 and -10, used only for camera tests).
- Order: gazebo with the world, one robot_state_publisher per URDF, spawn_entity.py for each robot, static map to odom (named flags), then the four nodes after a TimerAction delay. All processes use use_sim_time true.

### D11. Dependencies

numpy, cv2, cv_bridge. Yaw from quaternion by hand with atan2. No tf_transformations.

### D12. Mission report

On reaching the goal the controller logs one line: elapsed_s since first motion, distance_m, mine_time_s (cost >= 95 under the UGV in /shared_costmap), mud_time_s (cost 80), road_time_s (cost 5). Metrics always read /shared_costmap.

### D13. Debug image

terrain_mapper_node publishes /uav/debug_segmentation at 5 Hz: the camera frame with each sampled 8 x 8 block tinted by class (red, yellow, green, grey for dirt) and a text overlay with UAV x, y, yaw and per-class cell counts.

### D14. Verification tools

tools/check_costmap.py and tools/check_path.py are verification-only scripts outside the package. The no-comment rule applies.
