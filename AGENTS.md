# tactical_recon: Agent Rules

Cooperative UAV-UGV reconnaissance and terrain-aware navigation in Gazebo Classic 11. Course project worth 50% of credit, 10-day deadline. A reliable, repeatable A/B demo matters more than extra features.

## Hard rules

1. No comments and no docstrings in any Python, XML, URDF, SDF, launch, or Bash file. This covers `#` comments, `<!-- -->`, triple-quoted docstrings, license headers, TODO markers, and commented-out code. Permitted: the XML prolog `<?xml version="1.0"?>` and a single `#!/usr/bin/env bash` first line in Bash scripts. Python files have no shebang.
2. Ubuntu 22.04.5 LTS, ROS 2 Humble, Python 3.10, rclpy only. Never rospy or ROS 1 syntax.
3. Gazebo Classic 11 with gazebo_ros_pkgs only. Never gz-sim, Ignition, ros_gz_bridge, `gz sim`, or `ign`. Use `<sdf version="1.6">`.
4. The package is ament_python at ~/uav_ugv_ws/src/tactical_recon. No CMake and no C++ nodes.
5. OpenCV and cv_bridge come from apt (python3-opencv, ros-humble-cv-bridge). Never `pip install opencv-python`. Keep numpy below 2.
6. Values in docs/ARCHITECTURE.md sections 1 to 8 are locked. If one seems to cause a bug, stop and report it. Never change it silently.
7. Section 9 of docs/ARCHITECTURE.md holds the active design decisions (D1 to D14). Follow them unless docs/STATUS.md says one was changed.
8. Implement exactly one phase at a time as defined in docs/PHASES.md. Do not create files that belong to later phases.
9. Output complete files only. No placeholders, no ellipses, no pseudo-code.
10. Every node and every launch-started process uses use_sim_time=True.
11. OccupancyGrid.data is signed int8 in [-1, 100]. Never assign a value above 100 (for example 160 or 254); it crashes message packing.
12. setup.py must glob data_files for launch/*.launch.py, worlds/*.world, urdf/*.urdf, and rviz/*.rviz, or launch cannot find them after install.
13. Required plugins: libgazebo_ros_init.so and libgazebo_ros_factory.so in the world; libgazebo_ros_planar_move.so and libgazebo_ros_camera.so on the UAV; libgazebo_ros_diff_drive.so and libgazebo_ros_ray_sensor.so (output_type sensor_msgs/LaserScan) on the UGV.
14. If you are unsure of a gazebo_ros plugin parameter name, say so and write a test that proves it. Do not guess and continue.

## Environment

- Ubuntu 22.04.5, ROS 2 Humble, Gazebo Classic 11, AMD Ryzen 7 7435HS, 16 GB RAM, RTX 3050 4 GB (hybrid graphics: gzclient may need NVIDIA PRIME offload).
- Workspace: ~/uav_ugv_ws. Open this folder as the Antigravity workspace.
- Build and source:

```
cd ~/uav_ugv_ws && colcon build --symlink-install --packages-select tactical_recon && source install/setup.bash
```

- Run:

```
ros2 launch tactical_recon sim_mission.launch.py cooperative_mode:=true
```

- Clean stale simulators before every run:

```
pkill -9 gzserver; pkill -9 gzclient
```

## Workflow for every phase

1. Read AGENTS.md, docs/PRD.md, docs/ARCHITECTURE.md, docs/PHASES.md, docs/STATUS.md.
2. Create or edit only the files listed for the current phase.
3. Build, then run every verification step of that phase.
4. Show raw command output as evidence. Never claim a step passed without output.
5. Update docs/STATUS.md and stop. Do not start the next phase.

## File manifest

```
~/uav_ugv_ws/
├── AGENTS.md
├── docs/ (PRD.md, ARCHITECTURE.md, PHASES.md, STATUS.md)
├── tools/ (verification scripts, outside the package)
│   ├── check_costmap.py
│   └── check_path.py
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

setup.py must install launch, worlds, urdf, and rviz into share/tactical_recon and register four console_scripts, one per node.

## Topics and frames

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| /uav/cmd_vel | geometry_msgs/Twist | uav_patrol_node | planar_move plugin |
| /uav/odom | nav_msgs/Odometry | planar_move plugin | patrol, mapper |
| /uav/camera/image_raw | sensor_msgs/Image | camera plugin | mapper |
| /uav/patrol_done | std_msgs/Bool (transient_local) | uav_patrol_node | global_planner_node |
| /uav/debug_segmentation | sensor_msgs/Image | terrain_mapper_node | RViz |
| /ugv/cmd_vel | geometry_msgs/Twist | ugv_controller_node | diff_drive plugin |
| /ugv/odom | nav_msgs/Odometry | diff_drive plugin | mapper, planner, controller |
| /ugv/scan | sensor_msgs/LaserScan | ray sensor plugin | mapper |
| /shared_costmap | nav_msgs/OccupancyGrid | terrain_mapper_node | planner, controller, RViz |
| /lidar_costmap | nav_msgs/OccupancyGrid | terrain_mapper_node | planner, controller, RViz |
| /ugv/planned_path | nav_msgs/Path | global_planner_node | controller, RViz |

TF: map to odom is static identity. Plugins publish odom to uav_base_link and odom to ugv_base_link. Both costmaps use frame_id map.

## Locked numbers quick card

- Arena X, Y in [-15, 15]. UGV spawn (-12, -12, 0.2). Goal (12, 12). UAV spawn (-10, -10, 12), gravity 0, flies at Z = 12.0.
- Grid 150 x 150, 0.20 m/cell, origin (-15, -15, 0). gx = int((Xw + 15) / 0.2), index = gy * 150 + gx. Values int8: unknown -1, road 5, dirt 30, mud 80, mine or LiDAR wall 100 with 2-cell inflation.
- Camera 640x480 RGB8 10 Hz, hfov 1.047198, fx = fy = 554.256, cx = 320, cy = 240, pitched down rpy="0 1.570796 0". Pixel step 8.
- dx_body = -(v - cy) * 12 / fy, dy_body = -(u - cx) * 12 / fx, then rotate by UAV yaw and add UAV position.
- HSV: red H [0,10] or [170,180] S [100,255] V [80,255]; yellow H [18,34] S [100,255] V [80,255]; green H [35,85] S [80,255] V [50,255].
- A*: 8-connected, impassable at cost >= 95, step cost = euclid * (1 + cost / 10).
- Pure pursuit lookahead 0.8 m, stop within 0.5 m of goal. Speed governor v = 2.0 / (1 + cost / 10), see D9.
- UAV waypoints at 2.0 m/s: (-10,-10) (0,0) (-10,8) (8,8) (4,-4) (10,10).
- cooperative_mode is a boolean parameter on global_planner_node (and, by decision D9, on ugv_controller_node; default True).
