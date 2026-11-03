# Architecture & Engineering Decisions Log (DECISIONS.md)

This log records autonomous engineering decisions, rationales, and architectural choices made throughout the project lifecycle in accordance with the project autonomy guidelines.

---

## DECISION-001: Package Renaming and Canonical Layout
- **Date:** 2026-11-03
- **Decision:** Rename packages as follows:
  - `rt1_assignment2` -> `adaptive_speed_control` (Python ROS 2 package)
  - `rt1_assignment2_interfaces` -> `adaptive_speed_interfaces` (CMake ROS 2 interface package)
  - `bme_gazebo_sensors` -> `adaptive_speed_simulation` (CMake simulation package)
- **Rationale:** The legacy names were placeholders from university assignment templates. The new names reflect production robotics standards and accurately describe package responsibilities.
- **Impact:** All node executables, topic names, and launch files are organized cleanly under these namespaces. Backwards-compatible service aliases are retained where appropriate.

---

## DECISION-002: De-nesting Simulation Submodule into Primary Git Tree
- **Date:** 2026-11-03
- **Decision:** Remove the nested `.git` directory inside `bme_gazebo_sensors` before renaming to `adaptive_speed_simulation`, tracking all world models, meshes, and URDF files directly in the main repository.
- **Rationale:** The nested repository caused git submodule detachment, prevents atomic commits across worlds and controllers, and caused flake8 linter issues.
- **Impact:** Complete project version control in a single atomic repository.

---

## DECISION-003: Architectural Decoupling: UI/Navigator -> `/cmd_vel_in` -> Controller -> `/cmd_vel`
- **Date:** 2026-11-03
- **Decision:** Maintain `/cmd_vel_in` as the single ingress topic for user and autonomous navigation intent, and designate `adaptive_speed_controller_node` as the *exclusive* publisher to `/cmd_vel`.
- **Rationale:** Multiple velocity publishers competing on `/cmd_vel` cause actuator oscillation and safety hazards. Making the adaptive controller a mediator/filter guarantees safety invariants regardless of upstream intent.
- **Impact:** Upstream nodes (teleop, waypoint navigator) do not need complex collision math; the safety controller handles dynamic scaling and emergency stops uniformly.

---

## DECISION-004: Modularization of Speed Policy and Math Utilities
- **Date:** 2026-11-03
- **Decision:** Decouple all mathematical equations (distance factor, clutter factor, zone classification, and hysteresis) into `adaptive_speed_control.utils.speed_policy` and `adaptive_speed_control.utils.geometry_utils` as pure, testable Python functions without ROS 2 dependencies.
- **Rationale:** Pure functions can be tested exhaustively and rapidly in unit tests without launching node executors or ROS communication graphs.
- **Impact:** Fast test execution, high test coverage, deterministic mathematical validation.

---

## DECISION-005: Enhanced Interface Definitions
- **Date:** 2026-11-03
- **Decision:** Enrich `ObstacleInfo.msg` to include `front_distance`, `left_distance`, `right_distance`, `density_rho`, and `zone`. Introduce `RobotStatus.msg` with live metrics (`speed`, `commanded_speed`, `speed_limit`, `zone`, `state`, `distance_to_goal`, `distance_travelled`, `eta`).
- **Rationale:** Enables comprehensive telemetry, real-time dashboard display, and data logging for empirical comparison between fixed-speed and adaptive strategies.
- **Impact:** Full state observability for operators and research logging.

---

## DECISION-006: Headless Execution & Process Hygiene
- **Date:** 2026-11-03
- **Decision:** Execute Gazebo simulation headless (`gz sim -s -r --headless-rendering`) for all automated test runs, benchmarks, and verification steps. Enforce strict cleanup via process killing (`killall -9 gz-sim-server ruby gz` and `pkill -f ros2`) with explicit execution timeouts.
- **Rationale:** Prevents headless GUI hangs, X11 crashes, and orphaned simulation background tasks consuming system resources.
- **Impact:** High reliability across continuous integration and automated demonstration runs.

---

## DECISION-007: Real-Time Factor (RTF) Performance in Complex Worlds
- **Date:** 2026-11-03
- **Decision:** Validate the simulation RTF for Scenario 5 (`scenario_5_warehouse.sdf`, 20m x 14m) and Scenario 6 (`scenario_6_hospital.sdf`, 18m x 12m) using `gz sim -s -r --headless-rendering`.
- **Measurement:**
  - Scenario 5 (Warehouse): RTF measured at **1.0006 - 1.0008** (target $\ge 0.80$).
  - Scenario 6 (Hospital): RTF measured at **0.9996 - 1.0000** (target $\ge 0.80$).
- **Rationale:** Maintaining RTF near 1.0 guarantees that sensor physics, ray casting, and controller loop rates (20 Hz controller, 10 Hz navigator) reflect realistic real-time dynamics without clock dilation or missed watchdog deadlines.
- **Impact:** No reduction in LiDAR samples or update rates was necessary; the simple primitive geometric representations allow optimal ray-intersection efficiency.

---

## DECISION-008: Primitive Geometry vs Heavy Meshes for LiDAR Simulation
- **Date:** 2026-11-03
- **Decision:** Model all obstacles, warehouse shelf racks, cargo boxes, pallets, pillars, and dynamic pedestrians exclusively using analytic geometric primitives (`<box>`, `<cylinder>`, `<sphere>`), with Gazebo Fuel models completely optional and never required.
- **Rationale:** High-poly 3D meshes degrade ray-tracing throughput and introduce external internet download dependencies. Analytic primitives render deterministically offline, provide clean and predictable laser returns for the 2D LiDAR, and run flawlessly in air-gapped test environments.
- **Impact:** 100% offline simulation stability and sub-millimeter ray-casting accuracy.

---

## DECISION-009: Dynamic Obstacle Simulation via SetEntityPose vs Gazebo Actors
- **Date:** 2026-11-03
- **Decision:** Avoid using Gazebo Sim `<actor>` elements for dynamic pedestrians. Implement moving obstacles using `<model>` links possessing genuine `<collision>` tags, smoothly translated by `moving_obstacle_node` via the `/world/<world_name>/set_pose` service (`ros_gz_interfaces/srv/SetEntityPose`).
- **Rationale:** Gazebo actors frequently lack physical ray-tracing collision surfaces, making them invisible to `/scan` LiDAR sensors. Standard models with collision shapes dynamically transformed via `SetEntityPose` ensure full LiDAR detection fidelity.
- **Impact:** Robot detects moving people dynamically on `/scan`, slowing and soft-stopping appropriately.

---

## DECISION-010: Speed Visualization Modalities and Gazebo-Side Marker Evaluation
- **Date:** 2026-11-03
- **Decision:** Implement multi-modal speed visibility through:
  1. RViz 3D floating speedometer text marker (`/visualization_speedometer`) above the robot.
  2. RViz color-coded trajectory `LINE_STRIP` (`/visualization_trajectory`) with on-screen legend.
  3. Ground-level circular zone ring (`/visualization_zone_ring`).
  4. Live interactive matplotlib dashboard (`speed_dashboard_node.py`) with automatic headless protection.
  5. ANSI-colored terminal HUD from `odometry_monitor_node`.
- **Evaluation of Gazebo-Side 3D Text Marker:** Gazebo transport text markers published via `/sensors/marker` were evaluated. While functional in full GUI rendering, they are ignored by the headless rendering backend and do not reliably persist across client disconnects. Consequently, RViz text markers and the live matplotlib dashboard are designated as the canonical visualizers.
- **Impact:** Operators have instant visual awareness of speed, safety margin, and active zone in RViz, GUI, and console.

