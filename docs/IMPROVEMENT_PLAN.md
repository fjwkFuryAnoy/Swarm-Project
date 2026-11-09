# IMPROVEMENT PLAN (v2)

Goal: turn a working demo into a defensible project. A reviewer should conclude that the UAV discovers terrain, that the UGV drives a real obstacle course, and that the result holds on layouts nobody tuned for.

Baseline to protect: tag the current state `v1-submission` before touching anything. Every stage below must leave the project in a demo-ready state, so work can stop after any stage.

## 1. Problems this plan fixes

| # | Problem | Why a reviewer will notice |
|---|---|---|
| P1 | Terrain is pure flat colors, and the HSV bands were tuned to exactly those colors | The "recon" is a color lookup, so any learned model trained on this one world would just learn the same lookup |
| P2 | The green road runs from start to goal | It reads as a drawn guide line, not as terrain |
| P3 | The patrol waypoints fly over the known hazard positions | The UAV is told where to look, so nothing is discovered |
| P4 | One fixed layout, 3 identical runs (std 0.15 s) | Shows determinism, not generalization |
| P5 | The course is mostly empty (4 blocks, 2 mines, 1 mud patch) | It does not read as an obstacle course |
| P6 | Plain boxes, no shadows, flat lighting, flat colors | Aesthetics |
| P7 | No unit tests, `colcon test` fails, hard-coded `/home/fury/...` paths | Reproducibility |

## 2. Decisions (resolved)

| Gate | Decision | Effect on the plan |
|---|---|---|
| G0 | Time left is not a constraint | All stages are in scope. They still run in order, and each one leaves the project demo-ready |
| G1 | No neural network | Stage 3 uses a classical learned classifier (random forest on color and texture features). No CNN, no MLP, no GPU training |
| G2 | Accepted as proposed | Both start modes are supported. `wait_for_patrol` defaults to true (current behavior). The benchmark also reports the concurrent mode as its own row |
| G3 | Accepted as proposed | The road stays in-band green until Stage 3 passes. Stage 2 pass B then switches it to asphalt grey |

Stop-and-ask rule for any agent or session: if a result contradicts an acceptance check, stop and report the raw numbers. Do not change a threshold, a spec value, or a metric to make a check pass.

## 3. Order of execution

Stage 0, then 1, then 2 pass A, then 3, then 2 pass B, then 4. Pass B waits for Stage 3 because realistic textures break the HSV classifier by design. Estimates in the stage headings are guesses based on the size of the existing code.

---

## Stage 0: Safety net and hygiene (0.5 day)

Behavior must not change. Regression check: `run_benchmark.py` reproduces the README numbers within 5%.

1. `git tag v1-submission`. Work on a branch `v2`.
2. Replace every `/home/fury/College/Robotics/uav_ugv_ws` path in `run_baseline.sh`, `run_cooperative.sh`, and `tools/run_benchmark.py` with a path derived from the script location, or from `$UAV_UGV_WS`.
3. Fill in `package.xml` (description, license MIT, real maintainer email) and `setup.py`.
4. Delete `test_copyright.py` and `test_pep257.py`. They demand license headers and docstrings, which contradict the no-comment rule. Configure flake8 with a 100-character limit.
5. Create `tactical_recon/grid_utils.py` with the grid constants, world/grid conversion, and `quaternion_to_yaw`, currently duplicated across nodes. Import it everywhere.
6. Promote magic numbers to ROS parameters: lookahead, v0, acceleration limits, goal, patrol waypoints.
7. Add pytest unit tests: A* on small hand-made grids, projection round trip (pixel to world to cell), world/grid conversion, inflation.

Done when: `colcon test` passes, scripts run from a fresh clone, the benchmark matches v1.

---

## Stage 1: Make the recon real (1 to 2 days)

### 1a. Seeded course generator

`tools/gen_course.py --seed N` writes `worlds/course_N.world` and a ground-truth grid `gt_N.npy` (150 x 150, same encoding as the costmap, no inflation).

Layout rules:
- Fixed start (-12, -12) and goal (12, 12).
- Terrain classes: dirt, road, mud, sand (cost 55, new), mine (lethal), crater (lethal).
- Obstacles with collision: wall chicanes (at least 2 gates), rubble clusters, concrete blocks, sandbag lines.
- Roads form a short network with segments that do not connect start to goal. Include at least one decoy that leads into a dead end.
- A mine belt crosses the straight line between start and goal.
- Solvability: flood fill on the ground-truth grid with a 1.2 m minimum corridor width must find a path, otherwise resample.
- Diversity: store the ground-truth optimal path length and the straight-line length, and reject seeds where the optimal path is within 5% of straight.

### 1b. Coverage patrol (independent of hazard positions)

Three lanes at x = -10, 0, 10, sweeping y between -8.2 and 8.2:
(-10,-8.2), (-10,8.2), (0,8.2), (0,-8.2), (10,-8.2), (10,8.2).
At Z = 12 the camera footprint is 10.39 m along X and 13.86 m along Y, so these lanes cover the whole arena with about 0.4 m overlap. Route length is about 68 m, around 34 s at 2 m/s. Waypoints come from a parameter, not a hard-coded list. Do not stop at intermediate waypoints (blend with a look-ahead point), which cuts about 10 s versus the current approach.

### 1c. Mission modes

Add launch argument `wait_for_patrol` (default true). With false, the UGV starts at once and replans every 0.5 s as the map reveals. Both settings must work for every generated seed.

### 1d. Evaluation harness

`tools/run_benchmark.py --seeds 10 --modes baseline cooperative cooperative-concurrent` outputs CSV and a Markdown table with mean and std, plus per-seed pairs. `cooperative-concurrent` is cooperative mode with `wait_for_patrol:=false`.

Metrics:
- success (reached goal within timeout), time, distance
- exposure to each lethal class computed against `gt_N.npy` by footprint overlap (not against the inflated planning grid)
- exposure to slow classes
- map IoU per class at the moment the UGV starts moving
- minimum LiDAR range during the run (a near-collision proxy)

Report every seed, including seeds where cooperative does not win.

Done when:
- 10 generated seeds are all solvable and visibly different.
- At least 99% of non-dirt ground-truth cells are observed after the patrol.
- The benchmark runs unattended over 20 runs and writes the table.

---

## Stage 2: Aesthetics (2 to 3 days, two passes)

Do not mix the passes. After each one, rerun the Stage 1 benchmark and confirm the map IoU did not collapse.

### Pass A: HSV-safe look

Keeps the current classifier valid.
1. Ground: textured dirt (CC0 sources: Poly Haven, ambientCG) kept below saturation 100 so it never matches a band. Add a slight value variation.
2. Hazard patches: textures whose hue and saturation stay inside the existing bands, with irregular edges using alpha-masked planes.
3. Props: sandbag lines, concrete barriers, tank traps, ruined wall segments, containers. Collision on anything the UGV can hit, so LiDAR sees it. Decorative props without collision stay low and away from patches.
4. Lighting: enable shadows, a sky, a sun at about 45 degrees. Check the HSV classification still passes, since shadows lower V. If V drops below 80 on hazard pixels, raise ambient before touching the thresholds.
5. Robots: low-poly meshes (.dae) from Blender with real proportions: rover with sensor mast, LiDAR puck, antenna, tracks or six wheels; quadrotor with arms, motors, propellers, camera pod. Keep colors outside the HSV bands. Spinning propellers need a joint-velocity actuator in Gazebo Classic. Prototype before committing; fall back to static propellers if it takes more than half a day.
6. RViz and capture: fog-of-war reveal (already free from the -1 init), UAV camera footprint marker, tidy layout for a screen recording.

Budgets: textures at most 2048 px, scene under 500k triangles, real-time factor at least 0.8 with the GUI open (`gz stats`).

### Pass B: Full realism (after Stage 3)

Asphalt road (grey), realistic mud and mine fields, texture noise, hard shadows. HSV will fail here by design, so the mapper runs with `classifier:=forest`. This pass is only allowed once the Stage 3 acceptance checks pass.

---

## Stage 3: Learned perception, classical (1 to 2 days)

No neural network. A random forest on hand-built color and texture features.

Rule that prevents the "learns the green line" shortcut: the classifier only labels pixels with a terrain class. It never outputs a route or a cost map directly. Routing stays in A*. It is trained and tested on different generated layouts.

### 3a. Data
- Run the UAV over at least 60 generated seeds, saving frames at random positions at Z = 12.
- Labels are generated, not drawn: project each sampled pixel to the ground plane with the existing pinhole math and look it up in `gt_N.npy`.
- Randomize textures, sun angle, and shadows per seed.
- Split by seed, never by frame: 60% train, 10% validation, 30% test. Test seeds are never used for training or for tuning anything.

### 3b. Features and model
- Features for each sampled pixel (the same step-8 grid the mapper already uses): HSV of the pixel; mean and standard deviation of H, S, V over a 9 x 9 neighborhood; mean gradient magnitude; Laplacian energy. All computed with OpenCV and numpy.
- Model: scikit-learn random forest (about 100 trees, depth capped near 14), installed from apt (`python3-sklearn`) to respect the no-pip-for-vision rule. Save with joblib. Check class balance and weight the rare lethal classes.
- Inference cost must stay low: about 4800 samples per frame at 10 Hz.

### 3c. Integration
- The mapper takes a `classifier` parameter: `hsv` or `forest`. The costmap layers and hit-count voting stay unchanged.
- Pixels where the top class probability is below a threshold get a conservative cost (60) instead of the dirt cost (30). Tune the threshold on the validation seeds only.

### 3d. Evidence
- Per-class IoU and a confusion matrix on the unseen test seeds, for `hsv` and `forest`, on textured worlds.
- The Stage 1 benchmark rerun on the unseen seeds with each classifier.

Done when: the forest beats HSV on textured test worlds, mapper CPU stays below one core, and cooperative mode with the forest reaches the goal on at least the same fraction of test seeds as HSV did on the flat-color worlds.

---

## Stage 4: Hardening and presentation (0.5 to 1 day)

1. `colcon test` and all pytest tests pass on a clean clone.
2. Rewrite the README: new results (per-seed table, mean and std), the generated-course explanation, limitations, and future work.
3. Record a video: layout generation, UAV scouting with the fog-of-war reveal, baseline run, cooperative run, split screen.
4. Prepare the answers to three expected questions:
   - "Isn't the green line just the answer?" The road is a terrain class on a generated map. Decoy roads exist. The UGV leaves the road when that is cheaper. Results hold on unseen seeds.
   - "Why not learn the path end to end?" A model trained on one layout memorizes it. Here the learned part only classifies terrain, planning stays in A*, and everything is tested on unseen layouts.
   - "Is the result real or scripted?" Show the per-seed table, including seeds where cooperative loses or ties, and the ground-truth-based exposure metric.
5. State the known limitations plainly: 2D planner on flat ground, static world, kinematic UAV, no wind, no GPS error.

---

## Using Antigravity for each stage

Add the stage to `docs/STATUS.md` first, then use this prompt, one stage per conversation:

```
Read AGENTS.md, docs/PRD.md, docs/ARCHITECTURE.md, docs/IMPROVEMENT_PLAN.md and docs/STATUS.md. Implement Stage N only, section by section, and nothing from a later stage. Keep the no-comment rule. After each section run its acceptance check and show raw output. If a check fails, stop and report the numbers. Do not change thresholds, spec values, or metrics to make a check pass. Update docs/STATUS.md and stop at the end of the stage.
```
