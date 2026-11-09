# CODE REVIEW: Swarm-Project (tactical_recon)

Scope: everything in the repo at commit 98e43af (README, docs, four nodes, launch, setup, URDFs, world, RViz config, tools, scripts). I read the code and ran the planner's A* offline on synthetic grids. I did not run Gazebo, so nothing here is measured on your simulation.

Evidence column: **read** = seen in the code, **measured** = I ran it (on my machine, not yours), **inferred** = reasoned from the code.

## What is already good

- Timestamp-matched projection: the mapper queues images and interpolates the UAV pose at the image stamp (`get_interpolated_pose`) instead of using the latest odom. Most course projects skip this.
- Hit-count voting per cell (class beats dirt only on majority) is more robust than last-write-wins and absorbs edge blending.
- Consistent `use_sim_time`, correct transient_local QoS on costmaps, and the baseline/cooperative layer split in `/lidar_costmap` versus `/shared_costmap`.
- The terrain speed governor matches the planner's cost model, so the cooperative result is internally consistent.
- Clean separation into four nodes, a launch contract with a start delay, and a benchmark runner that kills stale processes.

## Findings

Severity: High = can lose credit or break reproduction. Medium = weakens results or robustness. Low = polish.

| ID | Sev | Area | Finding | Evidence | Fix |
|---|---|---|---|---|---|
| R1 | High | Validity | The recon is a color lookup. HSV bands match the world's flat colors exactly and the patrol flies over the known hazard positions. | read: `uav_patrol_node.waypoints`, `terrain_mapper_node.process_image` | Stage 1 and Stage 3 of IMPROVEMENT_PLAN.md |
| R2 | High | Validity | Benchmark is 3 runs of one deterministic layout (std 0.15 s). It shows repeatability, not generalization. | read: README table, `run_benchmark.py` | Seeded course generator and a 10-seed benchmark |
| R3 | High | Reproducibility | Absolute path `/home/fury/College/Robotics/uav_ugv_ws` hard-coded in `run_baseline.sh`, `run_cooperative.sh`, and `tools/run_benchmark.py`. | read | Derive from the script location or `$UAV_UGV_WS` |
| R4 | High | Tests | There are no unit tests. The three generated tests make `colcon test` fail by design: `test_pep257` requires docstrings and `test_copyright` requires license headers, which your no-comment rule forbids. flake8's default 79-column limit is exceeded in 49 lines. | read; line count by awk | Delete the two tests, set flake8 to 100 columns, add pytest tests for A*, projection, grid conversion |
| R5 | Medium | Planner | Corner-cutting check blocks a diagonal only when both orthogonal neighbors are blocked (`and`). Cutting past a single blocked corner is allowed, so the README claim "prevents diagonal corner cutting" is only half true. The 2-cell inflation hides it today. | read: `astar`, diagonal block | Block when either neighbor is blocked (`or`), then rerun the path check |
| R6 | Medium | Planner | A full replan from scratch every 0.5 s with no hysteresis, so near-equal-cost routes can flip between replans and the controller chases a jumping path. | read: `plan_timer` | Keep the previous path unless the new cost is lower by a margin (about 5%), or when the old path becomes blocked |
| R7 | Medium | Planner | A* runs inside the timer callback and blocks the single-threaded executor. Time per plan was 0.06 to 0.12 s on open and chicane grids, so up to about 25% of the executor at 2 Hz. Odometry and costmap callbacks wait while it runs. | measured | Fine for now. Move planning to a thread or `MultiThreadedExecutor` before making the course larger |
| R8 | Medium | Metrics | `mine_time_s` is read from the planning grid, which includes the 2-cell (0.4 m) red dilation and LiDAR inflation, and samples only the cell under the robot center. It overstates exposure and ignores the footprint. | read: `ugv_controller_node.control_loop` | Compute exposure from ground-truth geometry by footprint overlap |
| R9 | Medium | Controller | No local safety layer. Collision avoidance depends entirely on the 2 Hz global replan, which is thin for an obstacle course. | read | Add an emergency stop: if the minimum LiDAR range in the heading cone is below about 0.35 m, zero the speed |
| R10 | Medium | Controller | `closest_idx` scans the whole path every tick (20 Hz), which can jump to a far segment when the path passes near itself. Lookahead, v0, limits, and goal are hard-coded, so no parameter sweeps or ablations. | read | Search a window around the last index; make constants parameters |
| R11 | Medium | Mapper | LiDAR endpoints use the latest UGV odom, not the scan stamp, and yaw is interpolated linearly without angle wrap. While rotating in place the smear can reach about 0.4 m at 10 m range. The image path is stamp-matched, the LiDAR path is not. | inferred from `scan_cb` and `get_interpolated_pose` | Reuse the pose-queue interpolation for scans; unwrap yaw |
| R12 | Medium | Mapper | Hit counts never decay. Fine for a static world, but it silently breaks any dynamic obstacle or future moving hazard. | read | State it as a limitation, or add a decay factor |
| R13 | Low | Patrol | Proportional approach to each waypoint (speed = distance) decays exponentially, so each waypoint adds seconds before the 0.3 m tolerance is met. `patrol_done` is republished every 0.1 s after completion. | read | Blend waypoints with a look-ahead point; publish the latch once |
| R14 | Low | Structure | `quaternion_to_yaw` is copied into 3 nodes, and the grid constants (150, 0.20, -15.0) are repeated in all of them. | read | One `grid_utils.py` module |
| R15 | Low | Packaging | `package.xml` still says `TODO: Package description` and `TODO: License declaration`; `setup.py` has `fury@todo.todo`. README lists `opencv-python`, but the install step uses apt's `python3-opencv`. | read | Fill in and align |
| R16 | Low | Visuals | World has `<shadows>false`, ambient 0.8, flat-colored boxes, and no props. | read: `military_sector.world` | Stage 2 |
| R17 | Low | Baseline design | The baseline is deliberately terrain-blind. A reviewer may ask for a fairer baseline, for example a rover with an onboard forward camera, or a UAV map only partially complete. | inferred | Add a partial-coverage ablation: start the UGV after X% of the map is revealed |

## Suggested order of fixes

1. R3, R15, R14 (30 minutes, zero risk).
2. R4 and R5 together with unit tests, rerun the path check.
3. R1 and R2 through Stage 1 of the plan.
4. R8, R9, R10, R11 while the benchmark harness is being rewritten.
5. R6 and R7 only if the course gets bigger or the path visibly jitters.
