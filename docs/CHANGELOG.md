# Changelog

All notable changes to the **Adaptive Speed Control in Cluttered Environments** project are documented here.

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
