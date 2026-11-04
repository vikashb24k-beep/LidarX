# Codebase Audit Report (Phase 0)

**Date:** November 2026  
**ROS 2 Distribution:** Jazzy Jalisco  
**Simulation Environment:** Gazebo Harmonic (gz-sim 8.x)  
**Host Environment:** Ubuntu 24.04 LTS (Noble Numbat), Python 3.12.3  

---

## 1. Executive Summary

This audit assesses the state of the **LidarX** repository prior to refactoring and final polishing. The repository currently contains a functional prototype of a LiDAR-based adaptive speed controller with basic corridor waypoint following and emergency stopping, but retains legacy assignment package naming (`rt1_assignment2`, `rt1_assignment2_interfaces`, `bme_gazebo_sensors`), embedded submodule remnants, monolithic node implementations, and missing benchmark/metrics tooling.

---

## 2. Inventory of Existing Components

### 2.1 ROS 2 Packages
1. **`rt1_assignment2`** (Python package):
   - `distance_node.py` (~444 lines): Contains both the pure mathematical speed calculation function (`calculate_adaptive_speed`) and the ROS 2 node (`DistanceNode`). Subscribes to `/scan`, `/cmd_vel_in`, `/odom`; publishes `/cmd_vel`, `/obstacle_info`, `/obstacle_density`. Implements services `/set_threshold` and `/clear_barrier`.
   - `ui_node.py` (~176 lines): Interactive teleoperation UI running in a separate thread. Publishes to `/cmd_vel_in` at 10 Hz for timed duration. Implements `/get_average_vel` service with a rolling buffer of 5 commands.
   - `waypoint_navigator.py` (~183 lines): Centerline corridor guidance node following $y=0$ towards $(x=9.0, y=0.0)$ using odometry. Publishes nominal velocity commands to `/cmd_vel_in`.
   - `clear_barrier.py` (~52 lines): Helper utility invoking `gz service -s /world/empty/set_pose` to shift `zone4_barrier`.
   - Launch files: `adaptive_speed_control.launch.py` and `start_to_end_demonstration.launch.py`.
   - Tests: `test_adaptive_speed.py` (10 unit tests) and `test_node_integration.py` (4 integration tests).
2. **`rt1_assignment2_interfaces`** (CMake package):
   - `msg/ObstacleInfo.msg` (`float32 closest_distance`, `string direction`, `float32 threshold`).
   - `srv/SetThreshold.srv` (`float32 threshold` -> `bool success`, `string message`).
   - `srv/GetAverageVel.srv` (empty -> `float32 avg_linear`, `float32 avg_angular`, `int32 samples`).
3. **`bme_gazebo_sensors`** (CMake / Simulation package):
   - World: `worlds/my.sdf` (22m corridor with 4 environmental zones, floor markers, static obstacles, and a movable barrier).
   - URDF: `mogi_bot.urdf` differential drive robot with 2D planar LiDAR plugin.
   - Launch: `spawn_robot.launch.py` (bridges `/clock`, `/cmd_vel`, `/odom`, `/joint_states`, `/scan`) and `world.launch.py`.

---

## 3. Current Test and Build Results

### 3.1 Build Status
- Command: `colcon build --symlink-install`
- Result: **SUCCESS** (3 packages finished in ~18.5s).

### 3.2 Test Status
- Command: `colcon test --event-handlers console_direct+`
- Unit/Integration Tests (`rt1_assignment2`): **14 PASSED, 0 FAILED** in 0.49s.
  - `test_adaptive_speed.py`: 10/10 passed.
  - `test_node_integration.py`: 4/4 passed.
- Linter / Static Analysis (`bme_gazebo_sensors`):
  - `flake8`: 1 failure on legacy launch scripts (formatting/whitespace in upstream package).

---

## 4. Current Topic and Service Graph

```
[ User Teleop / UI Node ] ------+
                                |
[ Waypoint Navigator Node ] ----+---> /cmd_vel_in (Twist)
                                            |
                                            v
                                  [ Distance Node ] <--- /scan (LaserScan)
                                   (Adaptive Arbiter) <--- /odom (Odometry)
                                            |
                  +-------------------------+-------------------------+
                  |                         |                         |
                  v                         v                         v
          /cmd_vel (Twist)      /obstacle_info (ObstacleInfo)   /obstacle_density (Float32)
                  |
                  v
          [ Gazebo Bridge ]
                  |
                  v
         [ mogi_bot Simulation ]
```

### Services
- `/set_threshold` (`rt1_assignment2_interfaces/srv/SetThreshold`) - Hosted on `distance_node`.
- `/clear_barrier` (`std_srvs/srv/Trigger`) - Hosted on `distance_node`.
- `/get_average_vel` (`rt1_assignment2_interfaces/srv/GetAverageVel`) - Hosted on `ui_node`.
- `/start_navigation`, `/stop_navigation` (`std_srvs/srv/Trigger`) - Hosted on `waypoint_navigator`.

---

## 5. Deficiencies and Architectural Gaps

1. **Package Naming:** The workspace uses assignment names (`rt1_assignment2`, `rt1_assignment2_interfaces`, `bme_gazebo_sensors`) instead of production package names (`adaptive_speed_control`, `adaptive_speed_interfaces`, `adaptive_speed_simulation`).
2. **Coupled Mathematical Logic:** The adaptive speed equations, state machine transitions, and zone definitions are embedded directly in `distance_node.py` rather than modularized in an independent `utils/speed_policy.py`.
3. **Telemetry Limitations:** `ObstacleInfo.msg` lacks multi-sector distances (`front_distance`, `left_distance`, `right_distance`), computed density ($\rho$), and the active behavioral state/zone.
4. **Missing Diagnostics & Monitoring:** There is no dedicated `odometry_monitor_node` publishing comprehensive status (`RobotStatus.msg`) or `metrics_logger_node` logging quantitative navigation runs (speed, distance, time, jerk, proximity) for research benchmarking.
5. **Configuration Handling:** Key parameters are passed piecemeal across launch files rather than consolidated into clean YAML parameter files (`config/adaptive_speed_params.yaml`, `config/navigation_params.yaml`).
6. **Submodule Artifacts:** `bme_gazebo_sensors` was nested with an internal `.git` repository, creating detached submodule state.
7. **README Discrepancies:** The previous README referenced 13 tests, lacked documentation for the waypoint navigator and scenario 4, and had outdated package references.

---

## 6. Action Plan Across Phases

- **Phase 1:** Cleanly rename packages to `adaptive_speed_control`, `adaptive_speed_interfaces`, and `adaptive_speed_simulation`. Restructure source tree, add `RobotStatus.msg`, enrich `ObstacleInfo.msg`, and configure clean YAML params.
- **Phase 2:** Refactor `waypoint_navigator_node.py` into a robust state-machine navigator with reactive obstacle nudging, stuck recovery, and service controls.
- **Phase 3:** Extract speed policy into `utils/speed_policy.py` with pure functions, acceleration rate limiters, low-pass filtering, and zone hysteresis.
- **Phase 4:** Implement `odometry_monitor_node.py` with one-line terminal HUD and `/robot_status` publisher.
- **Phase 5:** Build `metrics_logger_node.py`, benchmark comparison scripts, scenario 4 world updates, and automated evaluation tools with matplotlib plots.
- **Phase 6:** Implement comprehensive test suite (unit tests for speed policy, sector processing, navigator state machine, and rclpy integration).
- **Phase 7:** Rewrite documentation (`README.md`, `ARCHITECTURE.md`, `DECISIONS.md`, `CHANGELOG.md`).
