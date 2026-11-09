# PRD: Cooperative UAV-UGV Reconnaissance and Terrain-Aware Autonomous Navigation

Target platform: ROS 2 Humble and Gazebo Classic 11.

## 1. Core objective

Show that a cooperative aerial scout (UAV) and ground rover (UGV) outperform a standalone rover in unknown military terrain. A rover with only a 2D planar LiDAR detects vertical walls but is blind to flat surface hazards: minefields, deep mud, and paved tactical roads. The UAV flies at a fixed 12.0 m, classifies surface terrain from a downward RGB camera, writes the costs into a shared global OccupancyGrid, and lets the UGV plan the fastest path to the goal while avoiding hazards.

## 2. Arena

- 30.0 x 30.0 m arena centered at (0, 0), X and Y in [-15, 15], with 2.0 m perimeter walls.
- UGV spawn (-12, -12, 0.2), yaw 0. Goal (12, 12). UAV spawn (-10, -10, 12), yaw 0.
- Terrain zones are flat boxes 0.02 m thick at Z = 0.01, visible to the camera and with no collision:
  1. Minefield, red (0.9, 0.0, 0.0), on the diagonal shortcut around (0, 0) and (-4, -4). It traps a naive shortest-path rover.
  2. Deep mud, yellow (0.9, 0.75, 0.0), around (4, -5) in the lower-right corridor. It sits off the diagonal and penalises off-road detours.
  3. Tactical highway, green (0.0, 0.8, 0.0), an L-shaped corridor from (-12, -12) to (-12, 10) and across to (12, 12).
- 4 static concrete blocks 1.5 x 1.5 x 2.0 m, detectable by the UGV LiDAR.

## 3. Evaluation modes

The planner and the controller both take a boolean parameter cooperative_mode (default True).

- Mode A, baseline (False): the rover sees only LiDAR obstacles. It is terrain-blind, so it drives the diagonal at dirt speed and crosses the red minefield.
- Mode B, cooperative (True): the UAV maps mines (100), mud (80), and road (5) first. The UGV routes along the green corridor, drives faster on the road, and reaches (12, 12) without entering a mine cell.

## 4. Success criteria

| Metric | Baseline | Cooperative |
|---|---|---|
| Time in mine cells | greater than 0 s | 0 s |
| Time to goal | about 65 to 75 s | about 35 to 45 s |
| Reaches goal and stops within 0.5 m | yes | yes |
| Repeatable over 3 runs | yes | yes |

## 5. Constraints

- 10-day deadline, worth 50% of course credit. A repeatable A/B demo outranks extra features.
- Zero comments in code files. See AGENTS.md.
