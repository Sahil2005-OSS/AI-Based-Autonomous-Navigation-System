# AI-Based Autonomous Navigation System

An autonomous 2D navigation system built from first principles in Python, featuring discrete occupancy grid mapping, conservative circular footprint clearance inflation, heuristic A* path planning, line-of-sight path smoothing, kinematic unicycle motion control with Pure Pursuit tracking, deterministic dynamic obstacles, and isolated-world online replanning.

---

## Table of Contents
- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [What Makes This Project Technically Interesting](#what-makes-this-project-technically-interesting)
- [Project Structure](#project-structure)
- [Environment Setup & Installation](#environment-setup--installation)
- [Running the System (CLI Workflows)](#running-the-system-cli-workflows)
- [Reproducible Demonstration Walkthrough](#reproducible-demonstration-walkthrough)
- [Benchmark Evaluation Suite](#benchmark-evaluation-suite)
- [Automated Testing](#automated-testing)
- [Limitations & Operational Assumptions](#limitations--operational-assumptions)
- [Technical Documentation Hub](#technical-documentation-hub)

---

## Project Overview

Autonomous mobile robots operating in real-world spaces must balance discrete topological graph search against continuous non-holonomic kinematic constraints and spatial clearances. Real navigation failures typically emerge not from pathfinding algorithms themselves, but from discrepancies between point-mass assumptions, grid discretization artifacts, corner-cutting short-circuits, and moving obstacles.

This project implements a complete, self-contained, and deterministic autonomous navigation pipeline designed to solve these challenges:
1. **Geometric Footprint Safety:** Replaces point-robot simplifications with a configurable circular robot footprint ($r_{robot}$) and exact continuous swept-footprint collision checks.
2. **Conservative Corner Crossing:** Implements strict Bresenham/DDA line-of-sight checks that forbid diagonal corner-clipping between solid obstacles.
3. **Continuous Motion Tracking:** Employs a kinematic unicycle model ($v, \omega$) and a Pure Pursuit geometric tracking controller with smooth deceleration and lookahead steering.
4. **Deterministic Moving Obstacles:** Simulates periodic, closed-form dynamic obstacles parameterized strictly by simulated time (never wall-clock time).
5. **Isolated Online Replanning:** Detects remaining-route conflicts and safely triggers A* replanning via temporary isolated planning grids without mutating the static base map.
6. **Safe Halting Contract:** Replaces erratic recovery loops with guaranteed safe stopping (`ControllerStatus.BLOCKED`, $v=0, \omega=0$, zero collisions) when passages are blocked or replan budgets are exhausted.

---

## System Architecture

```text
                               +----------------------------------+
                               |     Scenario Configuration       |
                               |    (JSON: Arena, Obstacles,      |
                               |      Robot Radius, Speeds)       |
                               +-----------------+----------------+
                                                 |
                                                 v
                               +----------------------------------+
                               |           World Model            |
                               |  - Continuous/Grid Coordinate TX |
                               |  - Solid Shape Rasterization     |
                               |  - Footprint Clearance Inflation |
                               +-----------------+----------------+
                                                 |
                                                 v
                               +----------------------------------+
                               |         A* Path Planner          |
                               |  - 4/8-Connected Grid Topologies |
                               |  - Admissible Heuristics         |
                               |  - Corner-Cutting Prevention     |
                               +-----------------+----------------+
                                                 | Raw Path
                                                 v
                               +----------------------------------+
                               |       Path Post-Processing       |
                               |  - Conservative Line-of-Sight    |
                               |  - 85-95% Waypoint Reduction     |
                               |  - Swept Footprint Verification  |
                               +-----------------+----------------+
                                                 | Smoothed Path
                                                 v
                               +----------------------------------+
                               |    Pure Pursuit Controller       |
                               |  - Dynamic Lookahead Steering    |
                               |  - Heading Error Regulation      |
                               |  - Goal Slowdown Deceleration    |
                               +-----------------+----------------+
                                                 | Commands (v, omega)
                                                 v
       +--------------------+  Continuous Step   +------------------+
       |  Dynamic Obstacles | <----------------- | Kinematic Model  |
       |  - Closed-form p(t)|                    | - Unicycle Model |
       |  - Relative Swept  |                    | - Segment Swept  |
       |    Collision Check |                    |   Footprint Test |
       +---------+----------+                    +------------------+
                 |
    Route Conflict Detected?
    +------------+------------+
    |                         |
    v (Yes)                   v (No)
+-----------------------+   Continue Tracking
| Isolated Replanning   |
| - Temp Planning World |
| - A* Reroute          |
| - Swept Verification  |
| - Fail -> SAFE STOP   |
+-----------------------+
```

---

## What Makes This Project Technically Interesting

1. **Exact Synchronized Relative Continuous Motion:**
   Naive collision checks against moving obstacles check obstacle positions only at step endpoints, allowing fast-moving obstacles to "tunnel" through the robot's swept path between simulation steps. This system formulates the relative motion vector:
   $$\vec{P}(\lambda) = (R_0 - O_0) + \lambda\left[(R_1 - R_0) - (O_1 - O_0)\right] \quad \text{for } \lambda \in [0, 1]$$
   The exact minimum distance across the continuous timestep interval $[t, t + \Delta t]$ is computed via distance from the relative origin to the displacement segment, partitioned at obstacle turnaround points. Tunneling is mathematically eliminated.

2. **Conservative Corner-Crossing Safety:**
   Diagonal line-of-sight shortcutting algorithms often cut between two diagonally adjacent obstacle corners. This project enforces a conservative adjacent-cell test at every grid vertex crossing: both orthogonal adjacent cells $(gx + s_x, gy)$ and $(gx, gy + s_y)$ must be free and within world bounds.

3. **Isolated Temporary Planning Worlds:**
   Dynamic obstacles must be accounted for during replanning without polluting the permanent static map. The system generates cloned temporary planning worlds where moving obstacles are rasterized at their instantaneous positions with configured safety buffers ($\delta_{margin}$). Static world occupancy is preserved 100% immutable.

4. **Rigorous Telemetry & Time Separation:**
   Benchmark reporting strictly separates virtual physics simulation time ($t = steps \times \Delta t$) from CPU wall-clock computation time. Geometric path length is tracked separately from continuous distance traveled by the kinematic unicycle.

---

## Project Structure

```text
.
├── .gitignore                     # Git rules (excludes virtualenvs, caches; tracks evaluation reports)
├── README.md                      # Primary project portfolio and user documentation
├── requirements.txt               # Direct pinned runtime dependencies (pygame, numpy, matplotlib, pytest)
├── assets/                        # Static assets placeholder
├── config/                        # Scenario configurations
│   ├── default_scenario.json      # Primary 800x600 multi-obstacle arena
│   └── scenarios/                 # Deterministic benchmark scenarios
│       ├── blocked_unreachable.json
│       ├── dynamic_obstacle.json
│       ├── dynamic_obstacle_blocked.json
│       ├── insufficient_clearance.json
│       ├── obstacle_detour.json
│       └── open_world.json
├── docs/                          # Lightweight documentation hub
├── documentation/                 # Architectural specifications and execution logs
│   ├── architecture.md            # System architecture and mathematical formulations
│   ├── data_flow.md               # Pipeline data flow and transformations
│   ├── datasets.md                # Scenario definition schemas and configurations
│   ├── execution_and_validation_log.md # Milestones 01-11 engineering logs
│   ├── model_safety_and_risks.md  # Risk analysis and mitigations
│   └── modules.md                 # Detailed module and class contracts
├── results/                       # Automated benchmark output directory
│   ├── .gitkeep
│   └── evaluation_report.json     # Six-scenario reproducible benchmark metrics
├── scenarios/                     # Legacy scenario directory placeholder (.gitkeep)
├── src/
│   └── autonomous_nav/            # Core autonomous navigation package
│       ├── __init__.py
│       ├── config_loader.py       # JSON scenario loading and validation
│       ├── controller.py          # Pure Pursuit controller and simulation loop
│       ├── dynamic_obstacle.py    # Deterministic moving obstacles & relative collision
│       ├── evaluation.py          # Headless benchmark evaluation runner
│       ├── main.py                # Command-line entry point and dispatch
│       ├── motion_model.py        # Continuous kinematic unicycle motion model
│       ├── path_smoother.py       # Conservative line-of-sight path post-processor
│       ├── planner.py             # 4/8-connected A* graph search engine
│       ├── visualizer.py          # Pygame 2D interactive simulator
│       └── world.py               # 2D occupancy grid & swept-footprint checker
└── tests/                         # Automated test suite (163 passing tests)
    ├── __init__.py
    ├── test_basic_skeleton.py
    ├── test_config_loader.py
    ├── test_controller.py
    ├── test_dynamic_obstacles.py  # Moving obstacles, relative swept checks, replanning
    ├── test_evaluation.py         # Evaluation suite integrity and metric recording
    ├── test_footprint_clearance.py# Footprint inflation and swept-segment clearance
    ├── test_motion_model.py       # Kinematic unicycle motion model validation
    ├── test_path_smoother.py      # Line-of-sight shortcutting & corner safety
    ├── test_planner.py            # A* optimality, admissibility, and topology
    ├── test_visualizer.py         # Headless visualizer playback and telemetry
    └── test_world.py              # Grid transforms and geometric rasterization
```

---

## Environment Setup & Installation

### Prerequisites
- **Python:** 3.12 (or 3.11+)
- **OS:** Windows, Linux, or macOS

### Clean Installation from Source

1. **Clone or Navigate to the Workspace:**
   ```bash
   cd "AI BASED AUTONOMOUS NAVIGATED SYSTEM"
   ```

2. **Create a Local Virtual Environment:**
   - **Windows (PowerShell):**
     ```powershell
     python -m venv .venv
     ```
   - **Linux / macOS:**
     ```bash
     python3 -m venv .venv
     ```

3. **Activate the Virtual Environment:**
   - **Windows (PowerShell):**
     ```powershell
     .\.venv\Scripts\Activate.ps1
     ```
   - **Linux / macOS:**
     ```bash
     source .venv/bin/activate
     ```

4. **Install Dependencies:**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
   *Verified dependencies:* `pygame>=2.5.0`, `numpy>=1.26.0`, `matplotlib>=3.8.0`, `pytest>=7.4.0`.

5. **Verify Installation:**
   ```powershell
   $env:PYTHONPATH="src"; .\.venv\Scripts\pytest.exe -q
   ```
   *(Expected: `163 passed` in under 6 seconds).*

---

## Running the System (CLI Workflows)

All commands are executed from the project root. On Windows PowerShell, prefix with `$env:PYTHONPATH="src";`.

### 1. Baseline Path Planning & Smoothing (Headless)
Computes A* path, applies line-of-sight smoothing, and displays plan metrics:
```powershell
$env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py
```
- Bypass smoothing: `--no-smooth`
- Set 4-connected topology: `--connectivity 4`
- Override robot footprint radius: `--radius 20.0`
- Use custom scenario: `--config config/scenarios/obstacle_detour.json`

### 2. Continuous Kinematic Motion Tracking (Headless)
Runs the kinematic unicycle simulation with Pure Pursuit path tracking:
```powershell
$env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --motion
```

### 3. Dynamic Obstacle Scenarios
- **Detour Replanning Scenario:**
  Moving obstacle patrols the central corridor. The robot detects route conflict, creates an isolated temporary planning grid, finds a safe detour, and reaches the goal:
  ```powershell
  $env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --config config/scenarios/dynamic_obstacle.json --motion
  ```
- **Blocked Corridor Safe-Stop Scenario:**
  Corridor is completely blocked by static and dynamic obstacles. Replanning recognizes zero valid bypasses and halts cleanly:
  ```powershell
  $env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --config config/scenarios/dynamic_obstacle_blocked.json --motion
  ```

### 4. Reproducible Evaluation Benchmark Suite
Runs all six benchmark scenarios headlessly, validates safety metrics, prints the telemetry table, and writes machine-readable results to `results/evaluation_report.json`:
```powershell
$env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --evaluate
```

### 5. Interactive 2D Pygame Visualizer (GUI)
*Requires an environment with a connected display/desktop session.*
- **Continuous Motion with Dynamic Obstacles:**
  ```powershell
  $env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --config config/scenarios/dynamic_obstacle.json --visualize --motion
  ```
- **Discrete Waypoint Step Mode:**
  ```powershell
  $env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --visualize
  ```

#### Interactive Keyboard Controls
| Mode | Key | Action |
| :--- | :--- | :--- |
| **Motion Mode** | `[SPACE]` / `[P]` / `[A]` | Pause or resume continuous motion tracking |
| | `[R]` | Reset robot pose to start and dynamic obstacles to $t=0$ |
| | `[ESC]` / `[Q]` | Cleanly exit simulation window |
| **Discrete Mode** | `[SPACE]` | Advance robot marker by 1 waypoint |
| | `[P]` / `[A]` | Toggle continuous auto-playback |
| | `[R]` | Reset marker to start grid cell |
| | `[ESC]` / `[Q]` | Cleanly exit simulation window |

---

## Reproducible Demonstration Walkthrough

Follow this 3-step sequence to reproduce the core safety behaviors of the system:

### Step 1: Static Navigation Arena with Conservative Smoothing
```powershell
$env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --motion
```
- **What to observe:**
  - Discrete A* expands 1,846 nodes to find an 80-waypoint path.
  - Path smoother eliminates redundant turns, yielding 4 waypoints (**95.0% reduction**).
  - Continuous unicycle motion tracks the route over 602 steps (30.10 s simulated time) to reach the goal within tolerance ($4.99 \le 5.0$).
  - Swept-footprint checks report `0 collisions`.

### Step 2: Dynamic Obstacle Conflict & Safe Online Detour
```powershell
$env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --config config/scenarios/dynamic_obstacle.json --motion
```
- **What to observe:**
  - Initial plan attempts a straight trajectory through the central corridor.
  - As obstacle patrols into the route, continuous conflict check triggers online A* replanning via temporary grid isolation.
  - Replacement route reroutes around the moving obstacle.
  - Robot executes 3 replans, travels 439.8 units over 381 steps (19.05 s simulated time), and reaches the goal safely.

### Step 3: Blocked Passage Safe Halting (`BLOCKED`)
```powershell
$env:PYTHONPATH="src"; .\.venv\Scripts\python.exe src/autonomous_nav/main.py --config config/scenarios/dynamic_obstacle_blocked.json --motion
```
- **What to observe:**
  - Corridors are fully obstructed.
  - Replanner queries temporary planning world, finds 0 valid routes, and triggers safe halting contract.
  - Output confirms: `Simulation Status: BLOCKED`, `Steps Executed: 0`, `Linear Speed: 0.0 u/s`, `0 collisions`.

---

## Benchmark Evaluation Suite

The evaluation suite runs deterministically and exports a structured JSON report to [`results/evaluation_report.json`](results/evaluation_report.json).

### Evaluation Summary Table

```text
+--------------------------+------+-------+-----------+----------------+-------------+--------------+----------+---------+--------------+--------+
| Scenario Name            | Plan | Nodes | Plan Time | Waypoints      | Planned Len | Traveled Len | Sim Time | Replans | Outcome      | Safety |
+--------------------------+------+-------+-----------+----------------+-------------+--------------+----------+---------+--------------+--------+
| default_navigation_arena | PASS | 1846  | 118.82 ms | 80->4 (-95.0%) | 899.3 u     | 893.5 u      | 30.10 s  | N/A     | REACHED_GOAL | SAFE   |
| open_world_straight      | PASS | 15    | 1.43 ms   | 15->2 (-86.7%) | 198.0 u     | 193.0 u      | 12.65 s  | N/A     | REACHED_GOAL | SAFE   |
| obstacle_detour          | PASS | 158   | 8.31 ms   | 25->4 (-84.0%) | 263.6 u     | 257.9 u      | 14.30 s  | N/A     | REACHED_GOAL | SAFE   |
| blocked_unreachable      | FAIL | 126   | 8.50 ms   | N/A            | N/A         | N/A          | N/A      | N/A     | NOT_RUN      | N/A    |
| insufficient_clearance   | FAIL | 110   | 7.15 ms   | N/A            | N/A         | N/A          | N/A      | N/A     | NOT_RUN      | N/A    |
| dynamic_obstacle_detour  | PASS | 31    | 1.84 ms   | 31->2 (-93.5%) | 300.0 u     | 439.8 u      | 19.05 s  | 3/3     | REACHED_GOAL | SAFE   |
+--------------------------+------+-------+-----------+----------------+-------------+--------------+----------+---------+--------------+--------+

Summary: 6 evaluated | 4 navigation successes | 2 expected planning failures | 0 unexpected evaluation errors
```

### Metrics Categorization
- **`navigation_successes` (4):** Planned, smoothed, and tracked to goal safely under continuous kinematics.
- **`expected_planning_failures` (2):** Correctly rejected by A* due to physical barriers or footprint bottlenecks; handled gracefully without interrupting suite execution.
- **`unexpected_evaluation_errors` (0):** Execution or schema exceptions; strictly zero tolerated.

---

## Automated Testing

The codebase includes an extensive suite of **163 automated tests** covering geometric rasterization, coordinate transforms, heuristic admissibility, corner safety, footprint inflation, swept collision checking, kinematic motion stepping, and dynamic obstacle replanning.

Run all tests headlessly:
```powershell
$env:PYTHONPATH="src"; .\.venv\Scripts\pytest.exe -v
```

### Test Breakdown by Subsystem
| Test Module | Tests | Focus Area |
| :--- | :---: | :--- |
| `test_world.py` | 17 | Occupancy grid, coordinate transforms, shape rasterization |
| `test_planner.py` | 18 | 4/8-connected A*, heuristics, corner-cutting prevention |
| `test_path_smoother.py` | 19 | Line-of-sight shortcutting, conservative corner-crossing safety |
| `test_footprint_clearance.py` | 17 | Circular footprint inflation, continuous swept-footprint checks |
| `test_motion_model.py` | 9 | Kinematic unicycle model, velocity clamping, pure rotation |
| `test_controller.py` | 21 | Pure Pursuit lookahead geometry, slowdown, safe halts |
| `test_dynamic_obstacles.py` | 23 | Periodic trajectories, relative continuous collision, replanning |
| `test_evaluation.py` | 16 | Scenario suite execution, metric recording, schema resilience |
| `test_visualizer.py` | 9 | Headless Pygame display, frame stepping, telemetry |
| `test_config_loader.py` | 10 | JSON scenario parsing, schema validation, error handling |
| `test_basic_skeleton.py` | 4 | Package skeleton and baseline importability |
| **Total** | **163** | **100% Passing** |

---

## Limitations & Operational Assumptions

To maintain engineering integrity, the following assumptions and limitations are explicitly noted:

1. **Current-Position Conflict Evaluation vs. 3D Space-Time Planning:**
   Route conflict detection is evaluated against moving obstacles at their current simulated pose. While continuous motion during each integration step evaluates exact relative motion, the system does not solve a full 3D space-time ($x, y, t$) time-extended lattice or Velocity Obstacle (VO) cone.
2. **Deterministic Simulation Assumptions:**
   Obstacles follow deterministic, closed-form linear trajectories. Stochastic human motion, sensor noise, covariance estimation (e.g. Kalman filtering), and perception occlusion are not simulated.
3. **Safe Halting Invariant:**
   When replanning fails or the maximum replan budget is exceeded during an active conflict, the robot enters `ControllerStatus.BLOCKED` ($v=0, \omega=0$). No speculative polling or automatic unblocking recovery is executed without an external command.
4. **Unmodeled Physical Dynamics:**
   The motion model assumes ideal kinematic unicycle equations without wheel slip, ground friction, motor torque limits, or physical payload inertia.
5. **Interactive GUI vs. Automated CI:**
   The Pygame visualizer requires an active graphical window manager. Automated regression testing and evaluation benchmarks run headlessly without opening a physical window.

---

## Technical Documentation Hub

Authoritative technical documentation, architectural specifications, and execution logs are maintained in [`documentation/`](documentation/README.md). For standard GitHub directory compatibility, [`docs/`](docs/README.md) functions as a forwarding index pointing directly to the canonical documentation:

- [System Architecture](documentation/architecture.md): Formal component breakdown and coordinate spaces.
- [Data Flow Pipeline](documentation/data_flow.md): End-to-end data transformation pipeline.
- [Module Breakdown](documentation/modules.md): Complete public API reference and class signatures.
- [Dataset Specifications](documentation/datasets.md): JSON scenario schemas and parameter bounds.
- [Model Safety & Risks](documentation/model_safety_and_risks.md): Failure modes, safety policies, and mitigations.
- [Execution & Validation Log](documentation/execution_and_validation_log.md): Chronological record of milestones 01 through 13.
