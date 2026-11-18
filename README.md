<div align="center">

# LidarX

### Adaptive Speed Control for Mobile Robots in Cluttered Environments

**ROS 2 Jazzy | Gazebo Harmonic | Python 3.12**

[![Build](https://img.shields.io/badge/build-colcon-success-brightgreen)](#build-and-test)
[![Tests](https://img.shields.io/badge/tests-62%20passed-brightgreen)](#test-results)
[![License](https://img.shields.io/badge/license-MIT-blue)](#license)

</div>

LidarX is a complete ROS 2 project that combines:

- 2D LiDAR perception;
- footprint-aware obstacle clearance;
- adaptive and fixed-speed controllers;
- waypoint navigation;
- keyboard and timed teleoperation;
- LiDAR fault injection;
- collision monitoring;
- metrics logging;
- RViz and dashboard visualization;
- Gazebo Harmonic simulation.

The main goal is to allow a mobile robot to move quickly in open areas while automatically slowing down or stopping in narrow, cluttered, or unsafe environments.

---

## 1. Project in One Sentence

> The waypoint navigator or user requests a desired velocity, but the adaptive safety controller is the only node that sends the final velocity to the robot; it reduces or stops that request using LiDAR, robot footprint, distance, clutter, braking physics, and timeout checks.

This project does not rely on a single speed formula. It has a layered safety architecture:

1. Generate a motion request.
2. Arbitrate teleoperation and navigation.
3. Measure the environment with LiDAR.
4. Calculate obstacle clearance and clutter density.
5. Apply the selected speed policy.
6. Apply acceleration limits.
7. Stop immediately on critical safety conditions.
8. Publish diagnostics and telemetry.

---

## 2. The Most Important System Invariant

The project uses the **Velocity Arbiter Pattern**.

```text
Waypoint Navigator ----\
                        \
Keyboard Teleoperation ---> twist_mux ---> /cmd_vel_in
Timed Teleoperation ----/                        |
                                                 |
                                                 v
                                    Adaptive Safety Controller
                                                 |
                                                 v
                                               /cmd_vel
                                                 |
                                                 v
                                  Gazebo differential-drive robot
```

Only `adaptive_speed_controller_node` publishes the final `/cmd_vel` topic. Navigation and teleoperation publish requests but cannot bypass the safety controller.

This also prevents conflicting commands:

- the keyboard is able to override navigation;
- navigation can continue in autonomous mode;
- neither source can disable emergency stopping;
- stale LiDAR data causes a stop;
- stale motion requests cause a stop.

---

## 3. ROS 2 Packages

| Package | Responsibility |
|---|---|
| `adaptive_speed_control` | ROS nodes, safety controller, policies, navigation, teleoperation, monitoring, launch files, tests |
| `adaptive_speed_interfaces` | Custom `ObstacleInfo` and `RobotStatus` messages and ROS services |
| `adaptive_speed_simulation` | Gazebo worlds, robot URDF, sensors, models, and ROS-Gazebo bridges |

The workspace source is organized as:

```text
src/
├── adaptive_speed_control/
├── adaptive_speed_interfaces/
└── adaptive_speed_simulation/
```

---

## 4. ROS 2 Computation Graph

```mermaid
flowchart LR
    KP[Keyboard Teleoperation] --> KT[/cmd_vel_teleop]
    TI[Timed Teleoperation] --> TI_TOPIC[/cmd_vel_in]
    NAV[Waypoint Navigator] --> NAV_TOPIC[/cmd_vel_nav]

    KT --> MUX[twist_mux]
    NAV_TOPIC --> MUX
    MUX -->|/cmd_vel_in| CTRL[Adaptive Speed Controller]

    GZ[Gazebo Robot] -->|/odom| CTRL
    GZ -->|/scan_raw| FI[LiDAR Fault Injector]
    FI -->|/scan| CTRL

    CTRL -->|/cmd_vel| GZ
    CTRL --> OBS[/obstacle_info]
    CTRL --> DENS[/obstacle_density]
    CTRL --> LIMIT[/safe_speed_limit]

    OBS --> NAV
    OBS --> MON[Odometry Monitor]
    OBS --> COLL[Collision Monitor]
    ODOM2[/odom] --> NAV
    ODOM2 --> MON
    MON --> STATUS[/robot_status]
    STATUS --> LOG[Metrics Logger]
```

For the complete system, the simulated environment publishes `/scan_raw`. The fault injector is inserted between `/scan_raw` and `/scan`, allowing the project to test normal, noisy, delayed, and dropout sensor behavior without changing Gazebo or the controller.

---

## 5. Main Nodes

### 5.1 Adaptive Speed Controller

File: `src/adaptive_speed_control/adaptive_speed_control/adaptive_speed_controller_node.py`

This is the central safety node.

It:

- subscribes to `/scan`, `/odom`, `/cmd_vel_in`, and control-mode topics;
- calculates sector distances and robot-footprint clearance;
- selects the requested speed policy;
- applies emergency stopping and watchdogs;
- publishes final velocity to `/cmd_vel`;
- publishes obstacle, scale, clearance, TTC, and zone telemetry.

The controller is implemented as a lifecycle node and can be configured, activated, deactivated, cleaned up, or shut down.

### 5.2 Waypoint Navigator

File: `src/adaptive_speed_control/adaptive_speed_control/waypoint_navigator_node.py`

The navigator:

- loads a sequence of waypoints from YAML;
- calculates heading errors;
- generates linear and angular motion requests;
- detects if it is stuck;
- waits for dynamic obstacles;
- enters recovery behavior if movement stops;
- publishes `/cmd_vel_nav`;
- publishes mission progress and state messages to the terminal.

It does not publish directly to `/cmd_vel`.

### 5.3 twist_mux

`twist_mux` arbitrates between teleoperation and navigation:

| Input | Priority |
|---|---:|
| `/cmd_vel_teleop` | 100 |
| `/cmd_vel_nav` | 10 |

Keyboard teleoperation wins when active. Both streams are remapped to `/cmd_vel_in`, which is the controller input.

### 5.4 LiDAR Fault Injector

File: `src/adaptive_speed_control/adaptive_speed_control/scan_fault_injector_node.py`

It reads `/scan_raw` and publishes a possibly degraded `/scan`.

Supported faults include:

- Gaussian noise;
- proportional range noise;
- random dropout;
- delayed delivery;
- dropped scans;
- frozen scans;
- blinded angular sectors.

Profiles are stored in `config/fault_profiles.yaml`.

### 5.5 Odometry Monitor

File: `src/adaptive_speed_control/adaptive_speed_control/odometry_monitor_node.py`

It calculates:

- actual speed;
- cumulative travelled distance;
- distance to goal;
- ETA;
- active safety state;
- commanded speed and published limit.

It also publishes RViz markers for the speedometer, trajectory, safety ring, nearest obstacle, and legend.

### 5.6 Collision Monitor

File: `src/adaptive_speed_control/adaptive_speed_control/collision_monitor_node.py`

It:

- reads Gazebo contact data;
- counts debounced collision events;
- counts near misses;
- calculates minimum clearance;
- estimates hard brakes;
- records acceleration variance and RMS jerk.

### 5.7 Metrics Logger

File: `src/adaptive_speed_control/adaptive_speed_control/metrics_logger_node.py`

It writes trajectory and safety data to CSV.

Typical fields include position, speed, requested and commanded velocity, front and closest distance, density, zone, and travelled distance.

### 5.8 Teleoperation Nodes

- `keyboard_teleop_node.py` publishes hold-to-move commands at 20 Hz to `/cmd_vel_teleop`.
- `teleop_ui_node.py` accepts timed commands and publishes to `/cmd_vel_in`.
- `gazebo_key_bridge_node.py` converts Gazebo keyboard input into ROS 2 teleoperation commands.

---

## 6. How the Safety Controller Works

The controller processes each LiDAR scan and command as follows.

### Step 1: Receive a LiDAR scan

The node subscribes to `/scan` using sensor-data best-effort QoS.

For every valid laser range, it:

- filters `NaN`, `Inf`, negative, and out-of-range values;
- calculates the closest distance;
- divides the scan into left, front, and right sectors;
- computes sector minimum distances;
- calculates obstacle density.

Invalid readings do not become zero or an arbitrary obstacle.

### Step 2: Transform points into the robot footprint frame

The controller does not rely only on the minimum global LiDAR ray.

It converts each LiDAR point from the sensor frame to the robot base frame using the sensor and robot-footprint offsets.

The robot parameters include:

- chassis length;
- chassis width;
- front and rear offsets;
- sensor offset;
- lateral margin;
- circumscribed radius.

The robot is tested against a swept forward corridor rather than only as a point.

The swept corridor can be considered:

```text
front obstacle
       |
       v
  [ROBOT WIDTH]
  |<----------| lookahead corridor
```

The controller calculates forward clearance, reverse clearance, rotational clearance, closest point, and forward clutter density.

### Step 3: Calculate front clearance

By default, the forward clearance comes from footprint-aware corridor processing.

The effective clearance is intentionally conservative:

- the latest scan is recorded;
- a 3-scan history is maintained;
- a close obstacle seen in at least 2 of the 3 latest scans is retained;
- a single clean or empty scan cannot immediately erase a recently detected obstacle.

This prevents a single missing or misleading ray from causing an unsafe instant resume.

### Step 4: Apply an emergency stop

If effective clearance is less than or equal to the base threshold, the controller publishes a zero `Twist` and stops both linear and angular motion.

The default threshold is:

```text
threshold = 0.45 m
```

This is a hard safety boundary, not just a speed-reduction recommendation.

### Step 5: Check watchdogs

The controller checks two independent timeout conditions.

**LiDAR watchdog**

- incoming scan header age is checked against simulation time;
- receive age is also checked;
- if the scan is stale, the state becomes `SENSOR_TIMEOUT`;
- final command becomes zero.

**Command watchdog**

- if no `/cmd_vel_in` command arrives within 0.50 seconds;
- the controller enters `IDLE`;
- final command becomes zero.

Therefore, disconnected teleoperation or a stopped navigator does not leave the robot moving indefinitely.

### Step 6: Apply the selected speed policy

The default configuration uses the `layered_governor`.

The available policies are:

| Policy | Purpose |
|---|---|
| `fixed` | Baseline fixed speed with emergency stops |
| `distance_linear` | Linear distance-based slowdown |
| `density_only` | Slowdown based on obstacle density |
| `distance_density` | Combined distance and density policy |
| `distance_braking` | Physics-based braking-distance limit |
| `distance_density_braking` | Combined density and braking-distance policy |
| `layered_governor` | Multiple independent safety and comfort caps |

The layered governor combines:

- braking capacity;
- comfort distance;
- clutter density;
- TTC;
- lateral clearance;
- turn curvature;
- goal distance;
- creep speed;
- profile limits.

The final target is the minimum of the active caps:

```text
final target =
    min(requested speed,
        braking cap,
        comfort cap,
        clutter cap,
        TTC cap,
        lateral-width cap,
        turning cap,
        goal cap)
```

### Step 7: Apply acceleration limiting

Positive acceleration is smooth and bounded.

```text
v_next = min(v_target, v_current + max_acceleration * dt)
```

The default maximum acceleration is `1.0 m/s²`, and the controller runs at 20 Hz.

The system permits an instant decrease toward zero. Braking is not delayed to satisfy acceleration smoothing.

### Step 8: Publish final telemetry

The controller publishes:

| Topic | Data |
|---|---|
| `/cmd_vel` | Final linear and angular velocity |
| `/obstacle_info` | Front, left, right, closest distance, density, zone, scales |
| `/obstacle_density` | Normalized obstacle density |
| `/safe_speed_limit` | Current policy speed limit |
| `/diagnostics` | ROS diagnostics status |

---

## 7. LiDAR Sector and Density Model

The front sector spans approximately 60 degrees on each side of the robot:

```text
                front
          -60°  0°  +60°
            \   |   /
             \  |  /
              \ | /
               robot
```

For a 2D LiDAR:

```text
rho = number of close valid rays / number of valid rays
```

Where:

- close rays are within `density_radius = 1.30 m`;
- `rho` is between 0 and 1;
- sparse corridor side rays have a configurable deadband;
- density does not replace the hard distance stop.

The system uses density as a slow-down factor. It is not allowed to become the only source of truth for stopping.

---

## 8. Physics-Based Braking

The dynamic safety margin is:

```text
d_margin(v) =
    d_base
    + v * (reaction_time + scan_latency)
    + v² / (2 * braking_acceleration)
```

This means:

- a faster robot needs a larger stopping margin;
- a delayed LiDAR scan requires more distance;
- braking capability is included in the safety calculation;
- the margin grows before the robot reaches the 0.45 m hard threshold.

The layered controller also solves for the maximum speed that can stop before the base threshold using the measured scan latency.

---

## 9. Environmental States

The controller labels its environment and telemetry as:

| State | Meaning |
|---|---|
| `STARTING` | Controller has started before enough sensor data is available |
| `EMPTY_CORRIDOR` | High clearance and low clutter |
| `MODERATE_TRAFFIC` | Reduced clearance or moderate clutter |
| `CLUTTERED_CORRIDOR` | Tight corridor or high density |
| `FIXED_BASELINE` | Adaptive scaling disabled |
| `CRITICAL_PROXIMITY` | Effective clearance reached the hard threshold |
| `SENSOR_TIMEOUT` | LiDAR data is missing or stale |
| `IDLE` | No recent upstream command |
| `REVERSE` | Reverse request received |
| `ROTATING` | Pure angular command |
| `STOPPED` | No linear or angular motion requested |
| `DESTINATION_REACHED` | Corridor end condition reached |

These labels help a professor or operator understand why the robot is moving or stopping.

---

## 10. Waypoint Navigation

The waypoint navigator:

1. starts or waits according to `auto_start`;
2. receives `/odom`;
3. receives obstacle sectors from `/obstacle_info`;
4. advances through configured waypoints;
5. calculates heading to the active waypoint;
6. publishes linear and angular requests to `/cmd_vel_nav`;
7. detects stuck conditions;
8. detects dynamic blocking;
9. enters waiting or recovery states.

The main heading controller is:

```text
heading_error = normalize(target_heading - robot_yaw)
angular_request = heading_gain * heading_error
```

The output is clipped to a maximum angular command.

The robot slows or pivots when the heading error is large. This is separate from obstacle safety, which is enforced later by the controller.

### Stuck recovery

If position does not change for the configured stuck timeout:

1. reverse gently;
2. rotate toward the side with more LiDAR clearance;
3. resume navigation.

### Dynamic obstacle waiting

If an object blocks the front and both lateral sides are also limited:

1. publish zero velocity;
2. wait for the object to move;
3. resume if clearance returns;
4. enter recovery if the blockage remains until timeout.

---

## 11. Simulation Scenarios

The project contains six simulation presets.

| Scenario | World | Purpose |
|---|---|---|
| 1 | Open corridor | Verify full-speed behavior and destination arrival |
| 2 | Moderate traffic | Verify proactive slowdown |
| 3 | Cluttered passage | Verify tight-space and density behavior |
| 4 | Barrier / critical zone | Verify controlled stop and clearance recovery |
| 5 | Warehouse | Test aisles, racks, pallets, and dynamic workers |
| 6 | Hospital | Test reception furniture, pillars, and dynamic people |

Scenario files and navigation routes are in:

- `src/adaptive_speed_control/config/`
- `src/adaptive_speed_control/adaptive_speed_control/`
- `src/adaptive_speed_simulation/worlds/`

Scenario 5 uses a 0.70 m/s nominal request with several intermediate waypoints through the warehouse.

Scenario 6 uses a 0.70 m/s nominal request and waypoints leading to the hospital goal.

---

## 12. Build and Test

### Prerequisites

- Ubuntu 24.04;
- ROS 2 Jazzy;
- Gazebo Harmonic;
- `twist_mux`;
- ROS-Gazebo bridge and interfaces;
- Python packages including `pytest`.

### Build the workspace

```bash
cd /home/vhack/vikash/ros2-lidar-safety-controller

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For a focused build:

```bash
colcon build --symlink-install \
  --packages-select adaptive_speed_interfaces \
                   adaptive_speed_control \
                   adaptive_speed_simulation
source install/setup.bash
```

### Test Results

The current control package test suite contains 62 tests.

```bash
ROS_LOG_DIR=/tmp/ros-log \
colcon test --packages-select adaptive_speed_control \
  --event-handlers console_direct+

colcon test-result --verbose
```

The test suite was verified with:

```text
62 passed
0 failed
0 skipped
```

The tests cover:

- adaptive speed boundaries;
- layered governor behavior;
- collision-monitor logic;
- keyboard teleoperation;
- lifecycle behavior;
- limit reasons;
- manual integration;
- node integration;
- navigator state machines;
- sector and geometry calculations.

---

## 13. Running the Project

### Headless autonomous Scenario 1

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export GZ_LOG_PATH=/tmp/gz-log
export ROS_LOG_DIR=/tmp/ros-log

ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=1 \
  headless:=true \
  autostart:=true
```

This starts:

- Gazebo;
- robot and LiDAR;
- ROS-Gazebo bridge;
- twist mux;
- fault injector;
- safety controller;
- waypoint navigator;
- odometry monitor;
- collision monitor.

The verified Scenario 1 behavior was that the controller leaves the sensor-timeout state, receives LiDAR, reaches cruise speed in clear space, and the navigator reports destination reached.

### GUI mode

```bash
ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=1 \
  headless:=false \
  rviz:=true
```

### Warehouse

```bash
ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=5 \
  headless:=true
```

### Hospital

```bash
ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=6 \
  headless:=true
```

---

## 14. Fixed-Speed Baseline

To compare with a fixed-speed strategy:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=3 \
  adaptive_enabled:=false
```

In this mode:

- the robot ignores distance and density speed scaling;
- it requests the nominal fixed speed;
- emergency stopping remains active;
- watchdog stopping remains active;
- the result can be compared with adaptive operation.

---

## 15. LiDAR Fault Testing

Fault profiles can be selected at launch:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=3 \
  headless:=true \
  fault_profile:=mild_noise
```

Available profiles:

- `none`
- `mild_noise`
- `heavy_noise`
- `dropout_10`
- `delay_200ms`
- `combined_worst`

Example:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=3 \
  headless:=true \
  fault_profile:=delay_200ms
```

The `delay_200ms` profile retains original scan header timestamps, allowing the controller to detect that the received sensor data is stale even if a delayed scan continues to arrive.

---

## 16. Manual Driving

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch adaptive_speed_control manual_demo.launch.py \
  scenario:=1 \
  headless:=true
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run adaptive_speed_control keyboard_teleop_node
```

Keyboard controls include:

| Key | Function |
|---|---|
| `w` | Increase forward request |
| `s` | Reverse or reduce forward request |
| `a` / `d` | Turn left / right |
| `q` / `e` | Forward-left / forward-right |
| `x` or Space | Stop |
| `m` | Toggle adaptive scaling |
| `t` | Switch manual or autonomous mode |
| `+` / `-` | Change requested maximum speed |

Disabling adaptive scaling only disables proportional speed reduction. It does not disable emergency stopping or timeouts.

---

## 17. Metrics and Bag Recording

Record topics for offline analysis:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py \
  scenario:=3 \
  headless:=true \
  bag:=true
```

Replay a bag:

```bash
bash scripts/replay_bag.sh bags/<bag-directory>
```

Analyze a bag:

```bash
python3 scripts/analyze_bag.py bags/<bag-directory>
```

Summarize result CSV files:

```bash
python3 scripts/analyze_results.py results
```

Generate plots:

```bash
python3 scripts/plot_results.py
```

---

## 18. Important Topics

| Topic | Meaning |
|---|---|
| `/scan_raw` | Raw Gazebo LiDAR output |
| `/scan` | Fault-injected LiDAR input to the controller |
| `/odom` | Robot pose and measured velocity |
| `/cmd_vel_teleop` | User teleoperation request |
| `/cmd_vel_nav` | Navigator request |
| `/cmd_vel_in` | Arbitrated upstream request |
| `/cmd_vel` | Final safety-authorized actuator command |
| `/obstacle_info` | Sector distances and speed scales |
| `/obstacle_density` | Normalized clutter density |
| `/safe_speed_limit` | Controller speed cap |
| `/robot_status` | Speed, ETA, distance, state, and goal data |
| `/diagnostics` | Controller and sensor diagnostics |
| `/contacts` | Gazebo collision contacts |

---

## 19. Important Services

| Service | Function |
|---|---|
| `/set_threshold` | Change the base stopping threshold |
| `/clear_barrier` | Move the Zone 4 barrier outside the path |
| `/start_navigation` | Start autonomous navigation |
| `/stop_navigation` | Stop autonomous navigation |
| `/reset_navigation` | Reset the waypoint index |
| `/get_collision_summary` | Return collision and smoothness metrics |
| `/get_average_vel` | Return the last timed teleoperation averages |

---

## 20. Repository Structure

```text
.
├── README.md
├── docs/
│   ├── ARCHITECTURE.md
│   ├── CHANGELOG.md
│   ├── DECISIONS.md
│   ├── RUN_GUIDE.md
│   ├── TEST_REPORT.md
│   └── figures/
├── scripts/
│   ├── run_scenario.sh
│   ├── run_all_scenarios.sh
│   ├── replay_bag.sh
│   ├── analyze_bag.py
│   ├── analyze_results.py
│   └── plot_results.py
├── results/
└── src/
    ├── adaptive_speed_control/
    │   ├── adaptive_speed_control/
    │   ├── config/
    │   ├── launch/
    │   └── test/
    ├── adaptive_speed_interfaces/
    │   ├── msg/
    │   └── srv/
    └── adaptive_speed_simulation/
        ├── launch/
        ├── urdf/
        └── worlds/
```

---

## 21. Professor Presentation: Recommended Explanation

A short explanation could be:

> LidarX is a ROS 2 adaptive speed-control system. It uses a Gazebo differential-drive robot and 2D LiDAR. An autonomous waypoint navigator and a teleoperation node generate desired velocities, but they do not control the robot directly. Those requests pass through `twist_mux` to the adaptive safety controller. The controller reads LiDAR and odometry, calculates a robot-footprint-aware forward clearance and clutter density, and then evaluates the requested speed using a layered governor. The governor considers braking capacity, LiDAR latency, TTC, lateral clearance, turning, clutter, and goal distance. Positive acceleration is rate-limited, while a critical obstacle or stale sensor causes immediate zero velocity. The controller publishes the only final `/cmd_vel` command and provides status, diagnostics, RViz markers, metrics, and collision information. The project includes six Gazebo scenarios, fixed-speed baseline mode, fault injection, manual driving, bags, and automated tests.

### 30-second version

> The system separates "what I want to do" from "what is safe to do." The navigator or teleoperator requests motion, while the controller decides the maximum safe motion using 2D LiDAR. It is the only node publishing to the robot.

### 60-second version

> It starts with `/odom` and LiDAR. The LiDAR is checked for valid rays, obstacle sectors, clutter density, and clearance inside the actual robot footprint. The controller uses a watchdog and a layered speed governor to combine braking distance, latency, TTC, lateral width, turning, density, and goal distance. It always keeps the emergency stop and timeout protections independent of adaptive scaling. Motion passes through `twist_mux` and reaches the final safety controller. The project includes navigation, manual control, dynamic obstacles, LiDAR faults, collision monitoring, ROS diagnostics, metrics, RViz visualizations, and six Gazebo worlds.

---

## 22. Technical Interview Questions

### Why is `/cmd_vel_in` separated from `/cmd_vel`?

It separates requested intent from authorized actuator commands. Multiple upstream nodes can send requests, but only the safety controller publishes to the motor interface.

### Why is this safer than reducing speed in the navigator alone?

The navigator is a planning and guidance node. It may have bugs or stop publishing. A centralized controller protects the robot regardless of upstream behavior and also protects manual teleoperation.

### How does the footprint-aware clearance differ from a front-ray minimum?

A front-ray minimum ignores the robot width and swept path. Footprint processing transforms every LiDAR point relative to the robot and checks whether the point enters a corridor around the chassis.

### What happens if a single LiDAR scan reports a very large open distance?

The effective clearance uses a 3-scan history. A newly empty reading cannot immediately erase an obstacle seen in at least two recent scans.

### What happens if the LiDAR stops publishing?

The node enters `SENSOR_TIMEOUT` and publishes zero velocity.

### What happens if the navigator stops publishing?

The node enters `IDLE` after the command timeout and publishes zero velocity.

### What happens if adaptive scaling is disabled?

The fixed-policy behavior allows the nominal requested speed, but safety stops and watchdogs remain active.

### Why is positive acceleration limited?

Step acceleration can cause wheel slip, jerky movement, actuator stress, and inaccurate odometry. Positive speed changes are ramped.

### Why can braking be immediate?

For safety, an obstacle must stop the robot as quickly as possible. Braking does not wait for a smooth acceleration ramp.

### How are dynamic obstacles detected?

Gazebo models have real collision and LiDAR geometry. A moving-obstacle node changes their poses through Gazebo services. The same LiDAR pipeline that detects static objects detects the moving objects.

### Why are simple geometric primitives used?

Boxes and cylinders provide deterministic, offline ray intersections and avoid large mesh downloads. They also preserve real-time factor in headless Gazebo.

### How do you compare adaptive and fixed-speed control?

Run the same scenario with `adaptive_enabled:=true` and `adaptive_enabled:=false`, while recording `/cmd_vel`, `/odom`, `/obstacle_info`, collision information, and time-series metrics.

### How is the project tested?

Pure policy functions are tested with pytest. Node integration and state-machine behavior are also tested. The ROS workspace uses colcon build and colcon test.

### What has been fully verified?

The full workspace builds, the 62-test control suite passes, launch arguments load, and Scenario 1 was verified headlessly with the controller leaving sensor timeout, receiving scans, reaching safe cruise speed, and navigating to the destination.

### What should not be overclaimed?

The complete 2–6 scenario matrix, complete fault campaign, and deliberate positive-collision test should not be described as fully verified until those runs are repeated and stored as reproducible evidence.

---

## 23. Limitations and Future Work

The current project is strong as a prototype and research platform, but it has areas for further work:

- complete verification of all six scenarios;
- complete ablation results for all policies;
- complete fault-study and collision positive-control data;
- improve lifecycle integration for the waypoint navigator;
- complete one consistent simulated-time and wall-clock metrics system;
- add parameter validation for every governor value;
- validate reverse-footprint behavior with additional testing;
- add continuous-integration configuration;
- add hardware-in-the-loop and real-sensor testing.

---

## 24. License

This project is licensed under the MIT License.

---

## 25. Author

- **Vikash**, IIIT Dharwad
