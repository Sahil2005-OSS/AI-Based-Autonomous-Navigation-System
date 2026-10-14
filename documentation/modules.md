# Module Breakdown & Specifications

## 1. Module Inventory

### Module 1: Sensor Ingestion & Preprocessing
- **Role:** Device driver interfaces, timestamp alignment, and sensor synchronization.
- **Input:** Raw streams (Camera, LiDAR, Odometry).
- **Output:** Synchronized sensor frame packet.

### Module 2: Perception & Scene Understanding
- **Role:** Deep learning models for 2D/3D object detection, semantic segmentation, and road boundary classification.
- **Input:** Synchronized image and point cloud tensors.
- **Output:** Tracked obstacles, class labels, velocities, and occupancy grid.

### Module 3: Localization & State Estimation
- **Role:** Sensor fusion (Extended Kalman Filter / Factor Graph SLAM) for ego-vehicle state.
- **Input:** IMU, wheel odometry, GNSS, landmark observations.
- **Output:** Ego state tuple $(x, y, z, \text{yaw}, \text{pitch}, \text{roll}, v_x, v_y, \omega_z)$.

### Module 4: Path & Trajectory Planning
- **Role:** Global route computation (A* / Dijkstra) and reactive local planning (TEB / DWA / RL-based policy) for collision avoidance.
- **Input:** Global goal, ego state, static map, dynamic obstacle tracks.
- **Output:** Temporal trajectory waypoints $[(x_t, y_t, v_t, \theta_t)]$.

### Module 5: Vehicle Control & Motion Actuation
- **Role:** Tracking error calculation and control signal generation (PID, Pure Pursuit, or Model Predictive Control).
- **Input:** Planned trajectory vs. current ego pose.
- **Output:** Normalized steering, throttle, brake commands.

---

## 2. Core Environment & World Subsystem (`autonomous_nav.world`)
- **Discrete Occupancy Grid:**
  - Continuous World: $(x, y)$ in $[0, \text{width}) \times [0, \text{height})$.
  - Grid Coordinates: $(gx, gy)$ in $[0, \text{grid\_width}) \times [0, \text{grid\_height})$.
  - Array Mapping: `grid[gy, gx]` (`(rows, cols)` indexed with `CELL_FREE=0`, `CELL_OCCUPIED=1`).
  - Resolution: Cell size $R$ in continuous units per grid cell.
  - Obstacle Geometry: Circle and Rectangle rasterization with continuous point collision checks.
  - Position Validation: Strict bounds and occupancy checking for agent start and goal locations.

---

## 3. Path Planning Subsystem (`autonomous_nav.planner`)
- **Core Planner:** [`AStarPlanner`](../src/autonomous_nav/planner.py#L64-L248) operating over the discrete 2D occupancy grid.
- **Connectivity Modes & Movement Costs:**
  - **4-Connected Mode:**
    - Allowed moves: Orthogonal $(\Delta x, \Delta y) \in \{(0, \pm 1), (\pm 1, 0)\}$.
    - Transition cost: $1.0$.
    - Heuristics: `manhattan` (default, admissible & consistent), `euclidean` (admissible).
  - **8-Connected Mode:**
    - Allowed moves: Orthogonal + Diagonal $(\Delta x, \Delta y) \in \{(\pm 1, \pm 1)\}$.
    - Transition cost: $1.0$ for orthogonal, $\sqrt{2} \approx 1.4142$ for diagonal.
    - Heuristic: `euclidean` (default, admissible).
    - Inadmissible rejection: `manhattan` is rejected in 8-connected mode because it overestimates true diagonal distances.
- **Diagonal Corner-Cutting Prevention:**
  - A diagonal step from $(x, y)$ to $(x + \Delta x, y + \Delta y)$ is forbidden if it passes between two blocked adjacent cells (`is_occupied(x + dx, y) and is_occupied(x, y + dy)`).
  - Optional `strict_diagonal_clearance=True` forbids diagonal moves if *either* adjacent cell is blocked.
- **Output Contract:**
  - [`PathResult`](../src/autonomous_nav/planner.py#L38-L53) dataclass: `success` (bool), `path` (list of `(gx, gy)` coordinates), `cost` (float, inf on failure), `expanded_nodes` (int), `message` (str).

---

## 4. Visual Simulation Subsystem (`autonomous_nav.visualizer`)
- **Core Visualizer:** [`SimulationVisualizer`](../src/autonomous_nav/visualizer.py#L48-L382) powered by Pygame.
- **Responsibilities:**
  - Converts discrete grid and world coordinates to scaled desktop pixel coordinates.
  - Renders 2D arena grid, obstacles, start/goal markers, and planned A* path polyline.
  - Manages interactive robot marker stepping (`step_forward`, `reset`, `toggle_autoplay`).
  - Renders real-time telemetry sidebar displaying plan status, cost, waypoint progress, expanded nodes, legend, and keyboard controls.
  - Provides headless-safe lifecycle methods (`render_frame`, `cleanup`) enabling zero-display CI/CD testing via `SDL_VIDEODRIVER="dummy"`.
- **Keyboard Interface:**
  - `[SPACE]`: Advance 1 waypoint.
  - `[P]` / `[A]`: Toggle auto playback.
  - `[R]`: Reset to start.
  - `[ESC]` / `[Q]`: Clean shutdown.

---

## 5. Path Smoothing Subsystem (`autonomous_nav.path_smoother`)
- **Core Smoother:** [`PathSmoother`](../src/autonomous_nav/path_smoother.py#L182-L308) executing greedy line-of-sight shortcutting.
- **Raycasting & Collision Check:**
  - Implements [`has_line_of_sight`](../src/autonomous_nav/path_smoother.py#L117-L212) using conservative 2D grid DDA traversal.
  - Evaluates all cells pierced by the continuous segment between cell centers.
  - Detects exact grid vertex crossings (`|t_next_x - t_next_y| < eps`) and identifies both adjacent orthogonal cells `(gx + sx, gy)` and `(gx, gy + sy)`.
  - Rejects line of sight if **either** adjacent cell is outside the grid bounds or occupied (`adj_x_blocked or adj_y_blocked`), enforcing conservative corner-clearance safety without permitting corner grazing.
- **Safety & Quality Guarantees:**
  - 100% collision-checked segments: No segment is accepted without verification against the occupancy grid.
  - Endpoint preservation: Original start and goal coordinates are strictly retained.
  - Monotonic waypoint ordering: Sequential path progression is preserved without loops.
  - Zero grid mutation: The underlying `World` and its matrix data remain unmutated.
  - Graceful fallback: If a direct shortcut cannot be guaranteed, original intermediate waypoints are retained.
- **Output Contract:**
  - [`SmoothedPathResult`](../src/autonomous_nav/path_smoother.py#L38-L57) dataclass: `success`, `original_path`, `smoothed_path`, `original_waypoints`, `smoothed_waypoints`, `original_cost`, `smoothed_cost`, `waypoints_removed`, `message`.

---

## 6. Kinematic Motion Model Subsystem (`autonomous_nav.motion_model`)
- **Core Model:** [`KinematicMotionModel`](../src/autonomous_nav/motion_model.py#L65-L167) implementing a discrete-time continuous unicycle model.
- **Robot State Representation:**
  - [`RobotState`](../src/autonomous_nav/motion_model.py#L38-L62) dataclass:
    - Continuous position: $x, y$ (world units).
    - Heading angle: $\theta$ (radians, normalized to $[-\pi, \pi)$).
    - Linear velocity: $v$ (world units/sec).
    - Angular velocity: $\omega$ (radians/sec).
    - Elapsed simulation time: $t$ (seconds).
  - Validation: Strict enforcement that all state coordinates, velocities, and timesteps are finite (`not isnan`, `not isinf`) and timestep $dt > 0.0$.
- **Equations of Motion:**
  $$\begin{aligned}
  x_{k+1} &= x_k + v_{cmd} \cdot \cos(\theta_k) \cdot \Delta t \\
  y_{k+1} &= y_k + v_{cmd} \cdot \sin(\theta_k) \cdot \Delta t \\
  \theta_{k+1} &= \text{normalize}\left(\theta_k + \omega_{cmd} \cdot \Delta t\right)
  \end{aligned}$$
- **Heading Normalization:**
  $$\text{normalize}(\theta) = ((\theta + \pi) \pmod{2\pi}) - \pi \quad \in [-\pi, \pi)$$
- **Velocity Clamping & Physical Limits:**
  - Forward velocity: $v \in [0.0, v_{max}]$ (or optional reverse bounds $[-v_{max}, v_{max}]$). Default $v_{max} = 40.0$ units/sec.
  - Angular velocity: $\omega \in [-\omega_{max}, \omega_{max}]$. Default $\omega_{max} = 2.0$ rad/sec ($\approx 114.6^\circ/\text{sec}$).
  - Acceleration bounding: optional acceleration limits $a_{max}$ and $\alpha_{max}$ preventing step discontinuities.

---

## 7. Pure Pursuit Path Tracking Controller (`autonomous_nav.controller`)
- **Core Controller:** [`PurePursuitController`](../src/autonomous_nav/controller.py#L93-L300) tracking collision-checked smoothed paths.
- **Target Selection & Lookahead Search:**
  - Input: Discrete smoothed path waypoints $[(gx_i, gy_i)]$.
  - Continuous Conversion: Maps grid centers to continuous coordinates using `world.grid_to_world(gx, gy)`.
  - Lookahead Target: Intersects a forward circle of radius $L_d$ (default $25.0$ units) with successive path segments from the current target index onward.
  - Final Goal Tracking: When the robot is within distance $L_d$ of the final goal waypoint, the goal itself is locked as the target.
- **Pure Pursuit Steering Law:**
  - Let target position be $(x_t, y_t)$ and robot pose $(x, y, \theta)$.
  - Angle to target: $\theta_t = \text{atan2}(y_t - y, x_t - x)$.
  - Relative heading error: $\alpha = \text{normalize}(\theta_t - \theta) \in [-\pi, \pi)$.
  - Lookahead distance: $L = \sqrt{(x_t - x)^2 + (y_t - y)^2}$.
  - Path curvature: $\kappa = \frac{2 \sin(\alpha)}{L}$.
  - Commanded angular velocity: $\omega_{cmd} = \text{clamp}(v_{cmd} \cdot \kappa, -\omega_{max}, \omega_{max})$.
- **Velocity Regulation & Goal Stopping:**
  - Approaching goal: If distance to goal $d_{goal} \le R_{slowdown}$ (default $40.0$ units), target velocity scales down linearly:
    $$v_{target} = \max\left(v_{min}, v_{max} \cdot \frac{d_{goal}}{R_{slowdown}}\right)$$
  - Heading error reduction: If $|\alpha| > \alpha_{threshold}$ ($45^\circ$), forward velocity is reduced proportionally to prioritize turning into the path heading before accelerating.
  - Goal condition: When $d_{goal} \le \text{tolerance}_{goal}$ (default $5.0$ units), controller transitions to `REACHED_GOAL`, setting $v_{cmd} = 0.0$ and $\omega_{cmd} = 0.0$.
- **Per-Step Conservative Collision Verification:**
  - Implemented in [`step_simulation`](../src/autonomous_nav/controller.py#L363-L440).
  - Before committing state update $S_{k+1}$, candidate continuous coordinates $(x_{next}, y_{next})$ are verified via `controller.check_step_collision(x_{next}, y_{next})`.
  - When `check_footprint=True`, checks full circular footprint clearance against boundaries and obstacle rectangles.
  - Safety rule: If the candidate position violates footprint clearance or world bounds:
    - Robot remains at last safe pose $S_k$ with $v=0, \omega=0$.
    - Controller status transitions immediately to `COLLISION`.
- **Headless Simulation Runner:**
  - [`run_simulation`](../src/autonomous_nav/controller.py#L442-L562) executes a bounded deterministic loop ($dt = 0.05$s, $N_{max} = 2000$), returning structured [`SimulationMetrics`](../src/autonomous_nav/controller.py#L70-L91).

---

## 8. Robot Footprint & Obstacle-Clearance Safety Subsystem (`Milestone 07 & 07.1`)
- **Footprint Model:**
  - Configurable circular robot footprint radius $R$ (default $15.0$ world units, configurable via scenario JSON `"robot": {"radius": 15.0}` or CLI `--radius <float>`).
  - Strict validation: non-negative, finite real numbers (`0.0 <= R < inf`, rejecting negative, NaN, and Inf).
- **Geometric Collision Checking (Static Endpoint vs. Swept Movement):**
  - **Endpoint Clearance (`check_footprint_collision`):** Evaluates a stationary circular footprint at position $(wx, wy)$, clamping $(wx, wy)$ to cell rectangle bounds $[x_1, x_2] \times [y_1, y_2]$ to check $(wx - cx)^2 + (wy - cy)^2 \le R^2$.
  - **Swept Movement Clearance (`check_swept_footprint_collision`):** Evaluates the entire continuous movement segment $S = [(x0, y0), (x1, y1)]$ carrying the circular footprint (swept capsule). Prevents obstacles from being skipped or tunneled through between simulation timesteps.
  - **Exact Segment-to-Rectangle Distance Algorithm:**
    - Intersection check: Tests whether segment $S$ intersects solid rectangle $R$ using ray-box slab intersection ($t \in [0, 1]$). If $S$ intersects $R$, distance is $0.0$.
    - Extreme-point distance minimization: For disjoint convex sets $S$ and $R$, the minimum Euclidean distance is mathematically guaranteed to be attained at:
      $$\text{dist}(S, R) = \min\big(\text{dist}(P_0, R), \text{dist}(P_1, R), \text{dist}(C_1, S), \text{dist}(C_2, S), \text{dist}(C_3, S), \text{dist}(C_4, S)\big)$$
      where $\{P_0, P_1\}$ are segment endpoints and $\{C_1, C_2, C_3, C_4\}$ are the four rectangle corners.
  - **Conservative Contact Policy:**
    - Tangential contact ($\text{distance} \le R$) is strictly rejected as a collision.
    - Boundary contact ($\min(x_0, x_1) - R \le 0$ or $\max(x_0, x_1) + R \ge \text{width}$, etc.) is strictly rejected as a collision.
- **Derived Clearance Map (`clearance_grid`):**
  - Stored alongside raw `grid` without mutating original obstacle data (`CELL_FREE=0`, `CELL_OCCUPIED=1`, `CELL_CLEARANCE_VIOLATION=2`).
  - Derived freshly via `update_clearance_grid()` on initialization and obstacle mutation, eliminating cumulative inflation errors.
- **Cross-Subsystem Clearance Integration:**
  - **A* Planner:** Expands only nodes where `world.is_footprint_safe_grid(gx, gy)` is True and enforces strict diagonal corner-clearance against clearance-violation cells.
  - **Path Smoother:** `has_line_of_sight(..., check_footprint=True)` validates endpoints, DDA ray-pierced cells, diagonal vertex crossings, and uses `world.is_swept_footprint_safe` along every candidate shortcut segment.
  - **Continuous Motion Controller:** Pure Pursuit `step_simulation` validates the proposed movement segment $(x_k, y_k) \to (x_{k+1}, y_{k+1})$ via `controller.check_step_collision(..., current_x=x_k, current_y=y_k)`. If swept clearance is violated, the proposed step is rejected, velocity stops ($v=0, \omega=0$), the last safe pose $(x_k, y_k, \theta_k)$ is preserved, and status transitions to `COLLISION`.
  - **Pygame Visualizer:** Highlights clearance zones in distinct muted indigo, outlines the physical robot footprint with an accent ring matching radius $R$, and reports footprint radius in telemetry.

---

## 9. Reproducible Evaluation & Benchmarking Subsystem (`Milestone 08`)
- **Module:** [`src/autonomous_nav/evaluation.py`](../src/autonomous_nav/evaluation.py)
- **Primary Roles & Public APIs:**
  - `compute_path_length_world(world, path)`: Calculates true Euclidean path length $\sum \sqrt{(wx_{i+1}-wx_i)^2 + (wy_{i+1}-wy_i)^2}$ in continuous world units.
  - `evaluate_scenario(...) -> ScenarioEvaluationResult`: Runs deterministic footprint-safe A*, line-of-sight path smoothing, and continuous kinematic motion simulation for a single scenario.
  - `run_evaluation_suite(...) -> EvaluationSuiteReport`: Aggregates batch evaluation across deterministic scenarios, outputs formatted ASCII comparison tables, and writes machine-readable JSON reports.
- **Data Models:**
  - [`ScenarioEvaluationResult`](../src/autonomous_nav/evaluation.py#L65-L144): Dataclass encapsulating planning, smoothing, motion, footprint safety telemetry, and explicit `evaluation_category` (`NAVIGATION_SUCCESS`, `EXPECTED_FAILURE`, `UNEXPECTED_ERROR`) for an individual run.
  - [`EvaluationSuiteReport`](../src/autonomous_nav/evaluation.py#L146-L270): Aggregates results across scenarios with timestamp, disclaimer, distinct 4-way counts (`navigation_successes`, `expected_planning_failures`, `unexpected_evaluation_errors`, `scenarios_evaluated`), backwards-compatible `scenarios_passed`/`scenarios_failed` aliases, and JSON serialization.
- **Benchmark Scenario Suite (`config/scenarios/`):**
  - `default_navigation_arena` (`config/default_scenario.json`): Baseline 800x600 arena with 3 obstacles.
  - `open_world_straight` (`config/scenarios/open_world.json`): 200x200 arena with 0 obstacles testing direct shortcutting.
  - `obstacle_detour` (`config/scenarios/obstacle_detour.json`): 300x200 arena with a vertical wall forcing an upward bypass detour.
  - `blocked_unreachable` (`config/scenarios/blocked_unreachable.json`): 200x200 arena with a solid vertical wall completely separating start and target.
  - `insufficient_clearance` (`config/scenarios/insufficient_clearance.json`): 200x200 arena with a 20-unit bottleneck rejecting a robot of radius 15.0 (diameter 30.0).
- **Reported Metrics & Units:**
  - `planning_success`: Boolean flag indicating whether A* found a footprint-safe path.
  - `planning_time_ms`: Wall-clock execution time for A* path search (milliseconds).
  - `expanded_nodes`: Priority-queue expansion count during A* search.
  - `original_waypoints` & `smoothed_waypoints`: Waypoint counts before and after smoothing.
  - `waypoint_reduction_pct`: Percentage reduction in waypoints $((N_{orig} - N_{smooth}) / N_{orig}) \times 100\%$.
  - `original_path_length_world` & `smoothed_path_length_world`: Planned path lengths in continuous world units.
  - `length_reduction_pct`: Percentage reduction in planned path Euclidean length.
  - `raw_path_footprint_safe`: Whether all raw A* waypoints and connecting segments pass continuous swept-footprint clearance.
  - `smoothed_path_footprint_safe`: Whether all smoothed shortcut segments pass continuous swept-footprint clearance.
  - `path_footprint_safe`: Composite flag (`raw_path_footprint_safe and smoothed_path_footprint_safe`).
  - `simulation_outcome`: Final status (`REACHED_GOAL`, `COLLISION`, `STEP_LIMIT`, or `NOT_RUN`).
  - `simulation_steps`: Count of discrete integration steps executed.
  - `simulated_time_s`: Virtual physics time elapsed ($steps \times \Delta t$ seconds).
  - `simulation_wall_time_ms`: Actual physical CPU duration of continuous motion simulation (ms).
  - `actual_distance_traveled_world`: Actual Euclidean distance traversed by the robot center during continuous tracking.
  - `final_distance_to_goal`: Final radial distance from robot center to target position.
  - `motion_footprint_safe`: Whether motion completed with zero collision occurrences.
  - `all_safety_checks_passed`: Composite safety indicator.
  - `evaluation_category`: Categorical tag (`NAVIGATION_SUCCESS`, `EXPECTED_FAILURE`, or `UNEXPECTED_ERROR`).
- **Critical Policy Distinctions & Integrity Checks (`Task 08.1 Audit`):**
  1. *Simulated Time vs. Wall-Clock Time:* Physics time in simulation is distinct from CPU execution duration.
  2. *Planned Path Length vs. Actual Distance Traveled:* Discrete/smoothed planned arc length is distinct from actual integrated trajectory length under non-holonomic kinematic steering.
  3. *Navigation Success vs. Expected Failure vs. Unexpected Error:* Expected planning failures (unreachable goals, clearance bottlenecks) are recorded cleanly without aborting the suite, while unexpected execution exceptions are not silently swallowed.
  4. *Swept Footprint Uniformity:* Both raw and smoothed path segments are verified using identical continuous swept-footprint collision logic.
  5. *Tested Run vs. Universal Safety Guarantee:* Absence of collision under deterministic unicycle simulation does not constitute a mathematical guarantee under dynamic obstacles, sensor noise, or unmodeled physical disturbances.

---

## 10. Dynamic Obstacles and Safe Replanning Subsystem (`Milestone 09`)
- **Module:** [`src/autonomous_nav/dynamic_obstacle.py`](../src/autonomous_nav/dynamic_obstacle.py)
- **Primary Roles & Public APIs:**
  - [`DynamicObstacle`](../src/autonomous_nav/dynamic_obstacle.py#L50-L268): Deterministic moving obstacle entity executing bounded linear patrol motion along $[p_A, p_B]$ as a pure function of virtual simulation time ($t$), strictly decoupled from wall-clock time.
  - [`point_to_segment_distance(px, py, x1, y1, x2, y2)`](../src/autonomous_nav/dynamic_obstacle.py#L19-L48): Exact orthogonal projection and endpoint-clamped Euclidean distance from continuous point to segment.
  - [`check_route_conflict(...)`](../src/autonomous_nav/dynamic_obstacle.py#L270-L320): Evaluates remaining path waypoints against active dynamic obstacles. Flags conflict if shortest distance $\le r_{rob} + r_{obs} + \delta_{margin}$.
  - [`create_temporary_planning_world(...)`](../src/autonomous_nav/dynamic_obstacle.py#L322-L368): Deep-clones static world, rasterizes dynamic obstacles at current pose expanded by $\delta_{margin}$, and regenerates clearance grid for A* planning without corrupting the permanent static map.
  - [`find_safe_start_grid(...)`](../src/autonomous_nav/dynamic_obstacle.py#L370-L400): Discretizes continuous robot pose to the nearest footprint-safe grid node to seed A* replanning.
- **Deterministic Trajectory Formulation:**
  $$\text{dist}_{AB} = \|p_B - p_A\|_2, \quad \text{period} = \frac{2 \cdot \text{dist}_{AB}}{\text{speed}}$$
  $$u = (\text{speed} \cdot t) \pmod{2 \cdot \text{dist}_{AB}}, \quad s = \begin{cases} u & \text{if } u \le \text{dist}_{AB} \\ 2 \cdot \text{dist}_{AB} - u & \text{if } u > \text{dist}_{AB} \end{cases}$$
  $$p(t) = p_A + \frac{s}{\text{dist}_{AB}} (p_B - p_A)$$
- **Safe Replanning State Machine & Fail-Safe Contract:**
  1. *Conflict Detection:* On each simulation step, checks remaining route waypoints against current obstacle position.
  2. *Cooldown & Rate Limiting:* Enforces configurable `replan_cooldown` (e.g. 0.5 s) and `max_replans` (e.g. 10) to prevent computational thrashing.
  3. *Isolated A* Search:* Plans detour through temporary planning grid.
  4. *Conservative Shortcut Smoothing:* Validates candidate replacement waypoints through line-of-sight and continuous swept-footprint clearance.
  5. *Route Substitution:* Updates controller active path only after complete validation.
  6. *Safe Stopping Halting Policy:* If replanning cannot find an alternate route (e.g. bottleneck blocked), the robot does **NOT** proceed along the unsafe route. It sets $v=0, \omega=0$, transitions to `ControllerStatus.BLOCKED`, and stops at a safe standoff distance with zero collisions.
- **Documented Theoretical Limitations:**
  - *Current Position vs. Future Trajectory:* The replanning trigger checks the dynamic obstacle's *current* position relative to the remaining route. It does not perform full spatiotemporal space-time planning ($x, y, t$) or velocity obstacle prediction. If an obstacle later moves into the replanned path, a subsequent replan is triggered. Full dynamic collision avoidance across fast-moving or adversarial agents requires predictive time-extended planning.
