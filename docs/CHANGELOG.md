# Changelog

All notable changes to the **Adaptive Speed Control in Cluttered Environments** project are documented here.

## [1.1.0] - Complex Worlds, Dynamic Obstacles & Visible Speed Displays (2026-11-03)

### New Worlds & Environments
- **Scenario 5: Warehouse (`scenario_5_warehouse.sdf`)**:
  - 20 m x 14 m industrial warehouse with concrete floor, yellow/black safety lines, and aisle signs.
  - 4 rows of tall shelf racks with orange uprights and brown cargo boxes, forming 1.7m wide aisles.
  - Scattered clutter: pallets, boxes, traffic cones, wet-floor sign, and a parked forklift.
  - 8 workers (cylinders: 0.5m x 1.7m), including 3 dynamic moving workers.
  - Real-Time Factor (RTF) measured headless at **1.0006**.
- **Scenario 6: Hospital Reception (`scenario_6_hospital.sdf`)**:
  - 18 m x 12 m healthcare lobby with light floor, doorway, and side corridor.
  - L-shaped reception desk, 3 rows of waiting chairs, architectural pillars, potted plants, wheelchair.
  - 8 people (staff in green, visitors in blue/purple/maroon), including 2 dynamic pedestrians.
  - Real-Time Factor (RTF) measured headless at **0.9996 - 1.0000**.

### Dynamic Moving Obstacles
- `moving_obstacle_node.py`:
  - Smoothly translates dynamic pedestrians/workers back and forth along 3D paths at 0.20 - 0.35 m/s.
  - Calls `/world/<world_name>/set_pose` via `ros_gz_interfaces/srv/SetEntityPose` over ROS-GZ bridge.
  - Verified headless: 2D LiDAR detects moving obstacles with exact geometric accuracy.
  - Dedicated YAML configs: `dynamic_obstacles_scenario_5.yaml` and `dynamic_obstacles_scenario_6.yaml`.
- Waypoint Navigator Soft-Stop Policy:
  - Detects dynamic frontal obstruction and enters `WAITING` soft-stop ($v=0, \omega=0$) for up to 4.0 seconds.
  - Resumes motion automatically when dynamic obstacle passes; triggers recovery only if path remains blocked.

### High-Visibility Speed Displays
- **RViz Floating Speedometer**: Text marker (`/visualization_speedometer`) floating above chassis showing `speed ... m/s | limit ... | zone ...` with 4-tier color coding.
- **Colour-Coded Trajectory Trail**: `LINE_STRIP` marker (`/visualization_trajectory`) colored by actual velocity (Green $\ge 0.28$, Yellow $\ge 0.18$, Orange $\ge 0.05$, Red $< 0.05$ m/s) with legend marker.
- **Ground Zone Ring**: Flat disk marker (`/visualization_zone_ring`) on floor beneath robot colored by safety zone (FREE, MODERATE, DENSE, EMERGENCY).
- **Speed Dashboard GUI** (`speed_dashboard_node.py`): Live matplotlib window with velocity gauge bar, rolling plot with zone background shading, and live numerical readouts. Features automatic headless detection without crashing.
- **ANSI Terminal HUD**: 1-2 Hz console readout in `odometry_monitor_node.py` using ANSI colors matching the speed and zone tiers.
- **Speed Profile Figures**: `plot_results.py` enhanced to plot world maps (SDF obstacles, shelves, workers) with overlaid velocity scatter and time-series speed profiles.

### Technical Interview Q&A
- Added 5 new viva questions (Q16-Q20) covering dynamic obstacles, simple geometry vs meshes for LiDAR, color calculation logic, soft-stop policies, and Gazebo actor limitations.

---

## [1.0.0] - Phase 1 Refactor & Restructuring (2026-11-03)

### Package Renames
- `rt1_assignment2` -> `adaptive_speed_control` (ROS 2 Python package).
- `rt1_assignment2_interfaces` -> `adaptive_speed_interfaces` (ROS 2 CMake interfaces package).
- `bme_gazebo_sensors` -> `adaptive_speed_simulation` (ROS 2 CMake simulation package).
- Removed nested `.git` repository from simulation directory to establish unified monorepo git tracking.

### New & Modularized Nodes
- `adaptive_speed_controller_node.py`: Production implementation of the adaptive speed controller node with multi-sector LiDAR parsing and rate-limited acceleration.
- `waypoint_navigator_node.py`: Autonomous corridor navigator featuring a complete state machine (`IDLE`, `ROTATE_TO_GOAL`, `DRIVE`, `AVOID_OBSTACLE`, `RECOVERY`, `GOAL_REACHED`, `FAILED`), cross-track error steering, and stuck detection.
- `teleop_ui_node.py`: Decoupled teleoperation interface publishing to `/cmd_vel_in` with average velocity tracking.
- `odometry_monitor_node.py`: Telemetry and HUD display node computing true Euclidean velocity, travelled distance, and ETA, publishing to `/robot_status`.
- `metrics_logger_node.py`: Quantitative benchmark logger writing time-series trajectory metrics to CSV.
- `clear_barrier.py`: Standalone CLI utility and ROS service client for Gazebo Harmonic dynamic barrier manipulation.

### Mathematical & Utility Modules
- `adaptive_speed_control.utils.speed_policy`: Pure functions for adaptive velocity scaling (`calculate_adaptive_speed`), LiDAR multi-sector distance parsing (`process_laser_scan_sectors`), and acceleration rate limiting (`rate_limit_velocity`).
- `adaptive_speed_control.utils.geometry_utils`: Coordinate transformations (`quaternion_to_yaw`, `normalize_angle`, `euclidean_distance`, `cross_track_error`).

### New Message & Service Interfaces
- Enriched `ObstacleInfo.msg`:
  - Added `front_distance`, `left_distance`, `right_distance`, `density_rho`, and `zone`.
- Added `RobotStatus.msg`:
  - `speed`, `commanded_speed`, `speed_limit`, `zone`, `state`, `distance_to_goal`, `distance_travelled`, `eta`.
- Retained `SetThreshold.srv` and `GetAverageVel.srv`.

### Launch & Configuration Files
- `config/adaptive_speed_params.yaml`: Centralized parameter definitions for the controller node.
- `config/navigator_params.yaml`: Centralized parameter definitions for the autonomous navigator.
- `launch/adaptive_speed_control.launch.py`: Launches standalone controller.
- `launch/start_to_end_demonstration.launch.py`: Launches controller, waypoint navigator, and odometry monitor.
- `launch/baseline_fixed_speed.launch.py`: Fixed-speed baseline demonstration for performance comparison.
- `launch/full_demo.launch.py`: Complete system launcher (Gazebo Harmonic, bridge, robot spawn, controller, navigator, HUD, and optional metrics logger).
# Unreleased — robustness integration

## QA follow-up

- Switched headless sensor configuration to CPU lidar and passed `use_sim_time` to control nodes.
- Documented the ROS/Gazebo LaserScan conversion blocker, observed test results, and verified run guide.

- Wired fault injection, collision monitoring, and twist mux into autonomous and manual launch paths.
- Routed keyboard/Gazebo key commands to `/cmd_vel_teleop` and waypoint navigation to `/cmd_vel_nav`.
- Added entry points, bag ignore rule, replay and offline CSV analysis utilities.
