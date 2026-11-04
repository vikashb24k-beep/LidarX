<div align="center">

# LidarX
### Adaptive Speed Control in Cluttered Environments
### ROS 2 Jazzy & Gazebo Harmonic Autonomous Mobile Robot Velocity Regulation

**A dynamic, reactive velocity regulation architecture for autonomous mobile robots in cluttered corridors using 2D LiDAR point clouds, spatial obstacle density estimation, and rate-limited actuation.**

<p>
  <img src="https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Ubuntu 24.04">
  <img src="https://img.shields.io/badge/Simulation-Gazebo_Harmonic-F58113?style=for-the-badge" alt="Gazebo Harmonic">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.12">
  <img src="https://img.shields.io/badge/Tests-26%20Passed-brightgreen?style=for-the-badge" alt="Tests">
  <img src="https://img.shields.io/badge/License-MIT-blue?style=for-the-badge" alt="License">
</p>

</div>

---

## 1. Executive Summary & Problem Statement

In autonomous ground robotics and remote teleoperation, fixed-speed navigation represents a fundamental trade-off dilemma:
- **High constant speed** optimizes transport throughput in wide, unobstructed corridors, but drastically elevates collision risk and braking distance inside tight, cluttered spaces.
- **Low constant speed** guarantees collision avoidance in tight passageways, but severely degrades mission efficiency in open transit zones.

This repository presents a complete, production-grade ROS 2 (Jazzy Jalisco) and Gazebo Harmonic solution that resolves this conflict. The system autonomously adapts the robot's translational speed in real time based on **forward obstacle proximity**, **spatial clutter density**, and **acceleration limits**, while enforcing deterministic emergency stops and seamless automatic resumption once obstacles are cleared.

---

## 2. System Architecture

The software architecture implements a strict **Velocity Arbiter Pattern**, ensuring that actuation commands to the robot chassis originate exclusively from the safety controller:

```mermaid
flowchart TD
    subgraph Guidance Layer
        Teleop["Teleop UI Node\n(teleop_ui_node.py)"]
        Nav["Waypoint Navigator Node\n(waypoint_navigator_node.py)"]
    end

    subgraph Mediator Topic
        CmdIn["/cmd_vel_in\n(geometry_msgs/Twist)"]
    end

    subgraph Arbiter & Safety Controller
        Controller["Adaptive Speed Controller Node\n(adaptive_speed_controller_node.py)"]
        Policy["Speed Policy Engine\n(utils/speed_policy.py)"]
        Controller -.->|evaluates| Policy
    end

    subgraph Actuator & Middleware
        CmdVel["/cmd_vel\n(geometry_msgs/Twist)"]
        Bridge["ros_gz_bridge\n(Parameter Bridge)"]
        Sim["Gazebo Harmonic Simulation\n(mogi_bot)"]
    end

    subgraph Feedback & Telemetry
        Scan["/scan (sensor_msgs/LaserScan)"]
        Odom["/odom (nav_msgs/Odometry)"]
        Info["/obstacle_info (ObstacleInfo)"]
        HUD["Odometry & HUD Monitor\n(odometry_monitor_node.py)"]
        Status["/robot_status (RobotStatus)"]
        Logger["Metrics Logger Node\n(metrics_logger_node.py)"]
    end

    Teleop -->|Publish Intent| CmdIn
    Nav -->|Publish Intent| CmdIn
    CmdIn -->|Input Intent| Controller

    Controller -->|Sole Actuation Publisher| CmdVel
    CmdVel --> Bridge --> Sim

    Sim --> Bridge --> Scan --> Controller
    Sim --> Bridge --> Odom --> Controller
    Bridge --> Odom --> Nav
    Bridge --> Odom --> HUD
    Bridge --> Odom --> Logger

    Controller --> Info --> HUD --> Status --> Logger
```

---

## 3. Mathematical Formulation

The adaptive speed controller continuously evaluates the forward LiDAR cone ($\theta \in [-60^{\circ}, +60^{\circ}]$) and scales translational velocity via two coupled attenuation factors:

### A. Distance Factor $D(d)$
Given the minimum obstacle distance $d$ within the front cone:

$$
D(d) = \begin{cases}
0.0, & d \le d_{safe} \\
\frac{d - d_{safe}}{d_{slow} - d_{safe}}, & d_{safe} < d < d_{slow} \\
1.0, & d \ge d_{slow}
\end{cases}
$$

- $d_{safe} = 0.45\text{ m}$: Critical safety threshold triggering immediate stop.
- $d_{slow} = 2.00\text{ m}$: Slowdown horizon initiating proportional deceleration.

### B. Clutter Density Factor $C(\rho)$
Surrounding obstacle density $\rho$ represents the fraction of valid LiDAR beams detecting obstacles within radius $R_{density} = 1.30\text{ m}$:

$$
\rho = \frac{N_{close}}{N_{valid}} \in [0.0, 1.0]
$$

$$
C(\rho) = \max\left(C_{min}, 1.0 - k_{dens} \cdot \rho\right)
$$

- $k_{dens} = 0.60$: Density influence coefficient.
- $C_{min} = 0.25$: Safety floor factor ensuring the robot does not stall purely from side clutter.

### C. Rate-Limited Commanded Velocity
The raw target velocity $v_{raw} = \min(v_{req}, v_{max}) \cdot D(d) \cdot C(\rho)$ is bounded by an acceleration rate limiter ($a_{max} = 0.60\text{ m/s}^2$) for smooth physical motion, while allowing immediate step braking for safety:

$$
v_{cmd}(t) = \begin{cases}
\min\left(v_{raw}, v_{cmd}(t - \Delta t) + a_{max} \Delta t\right), & v_{raw} > v_{cmd}(t - \Delta t) \\
v_{raw}, & v_{raw} \le v_{cmd}(t - \Delta t)
\end{cases}
$$

---

## 4. Environmental Zones & State Machine

The continuous 22-meter Gazebo corridor (`my.sdf`) is organized into four distinct zones:

```
[START (-9.0, 0.0)]                                                               [GOAL (9.0, 0.0)]
  |==== ZONE 1 ====||==== ZONE 2 ====||======== ZONE 3 ========||==== ZONE 4 ====||==== GOAL ====|
  | Open Corridor  || Moderate Traffic|| Cluttered Passage      || Critical/Barrier|| Arrival Pad |
  | (x: -11 to -5) || (x: -5 to -1)   || (x: -1 to +4)          || (x: +4 to +8)   || (x: +8 to +11)|
  | Speed: 0.40m/s || Speed: 0.25m/s  || Speed: 0.10 - 0.15 m/s || Speed: 0.00m/s  || Speed: 0.00m/s |
```

| Zone | Environment Character | Active State | Commanded Speed ($v_{cmd}$) | Primary Driver |
| :--- | :--- | :--- | :--- | :--- |
| **Zone 1** | Wide unobstructed corridor | `EMPTY_CORRIDOR` | $0.40\text{ m/s}$ (Nominal Max) | High path clearance, zero clutter ($\rho \approx 0$) |
| **Zone 2** | Staggered pillars | `MODERATE_TRAFFIC` | $0.20 - 0.28\text{ m/s}$ | Moderate distance and density ($\rho \approx 0.20$) |
| **Zone 3** | Dense boxes and cylinders | `CLUTTERED_CORRIDOR` | $0.10 - 0.15\text{ m/s}$ | High density ($\rho > 0.35$), tight lateral clearance |
| **Zone 4** | Transverse barrier at $x=6.6$ | `CRITICAL_PROXIMITY` $\to$ `CONTROLLED_STOP` | $0.00\text{ m/s}$ | Front distance $d \le 0.45\text{ m}$ |
| **Zone 4 Clear** | Barrier shifted to $y=10.0$ | `PATH_CLEAR` $\to$ `RESUME_NAVIGATION` | $0.20 \to 0.40\text{ m/s}$ | Hysteresis recovery ($d > 0.65\text{ m}$), auto-resume |
| **Destination** | Gold floor pad ($x=9.0$) | `DESTINATION_REACHED` | $0.00\text{ m/s}$ | Goal tolerance reached ($< 0.35\text{ m}$) |

---

## 5. Repository Structure

```
LidarX/
├── docs/
│   ├── ARCHITECTURE.md          # Complete architectural specification & graphs
│   ├── DECISIONS.md             # Autonomous engineering decision log
│   ├── CHANGELOG.md             # Package renames and version changelog
│   ├── CODE_AUDIT.md            # Initial codebase audit report
│   └── figures/                 # Generated benchmark evaluation plots
│       ├── adaptive_vs_fixed_benchmark.png
│       ├── adaptive_scenario_1_profile.png
│       └── adaptive_scenario_1_trajectory.png
├── results/                     # CSV logs of real simulation benchmarks
├── scripts/
│   ├── run_scenario.sh          # Headless automated scenario runner
│   ├── run_all_scenarios.sh     # Comprehensive benchmark evaluation script
│   └── plot_results.py          # Non-interactive matplotlib plotting engine
├── src/
│   ├── adaptive_speed_control/            # Main ROS 2 Python package
│   │   ├── adaptive_speed_control/
│   │   │   ├── adaptive_speed_controller_node.py # Main safety arbiter
│   │   │   ├── waypoint_navigator_node.py        # Autonomous corridor navigator
│   │   │   ├── teleop_ui_node.py                 # Teleoperation UI node
│   │   │   ├── odometry_monitor_node.py          # 1 Hz terminal HUD & status
│   │   │   ├── metrics_logger_node.py            # CSV time-series data logger
│   │   │   ├── clear_barrier.py                  # Barrier removal utility
│   │   │   └── utils/
│   │   │       ├── speed_policy.py               # Pure mathematical functions
│   │   │       └── geometry_utils.py             # Pure geometric transformations
│   │   ├── config/
│   │   │   ├── adaptive_speed_params.yaml        # Controller parameters
│   │   │   └── navigator_params.yaml             # Navigator parameters
│   │   ├── launch/
│   │   │   ├── full_demo.launch.py               # All-in-one demonstration launch
│   │   │   ├── start_to_end_demonstration.launch.py # Controller + Navigator + HUD
│   │   │   ├── adaptive_speed_control.launch.py  # Standalone controller
│   │   │   └── baseline_fixed_speed.launch.py    # Fixed speed baseline launch
│   │   └── test/                                 # 26 automated unit & integration tests
│   │       ├── test_adaptive_speed.py
│   │       ├── test_navigator_state_machine.py
│   │       ├── test_node_integration.py
│   │       └── test_speed_policy_comprehensive.py
│   ├── adaptive_speed_interfaces/         # ROS 2 message & service definitions
│   │   ├── msg/
│   │   │   ├── ObstacleInfo.msg
│   │   │   └── RobotStatus.msg
│   │   └── srv/
│   │       ├── SetThreshold.srv
│   │       └── GetAverageVel.srv
│   └── adaptive_speed_simulation/         # Gazebo simulation & URDF assets
│       ├── launch/
│       │   ├── spawn_robot.launch.py
│       │   └── world.launch.py
│       ├── urdf/mogi_bot.urdf
│       └── worlds/my.sdf
└── README.md
```

---

## 6. Quick Start & Build Instructions

### Prerequisites
- Ubuntu 24.04 LTS (Noble Numbat)
- ROS 2 Jazzy Jalisco
- Gazebo Harmonic (`ros_gz_sim`, `ros_gz_bridge`)
- Python 3.12 with `pytest` and `matplotlib`

### Building the Workspace
```bash
# Clone the repository
git clone https://github.com/your-username/LidarX.git
cd LidarX

# Source ROS 2 environment and build with colcon
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 7. Running the Demonstrations

### Robustness, footprint safety, and evaluation

Gazebo LiDAR is bridged to `/scan_raw`; `scan_fault_injector_node` publishes the controller-facing `/scan`. Select a reproducible profile with `fault_profile:=none|mild_noise|heavy_noise|dropout_10|delay_200ms|combined_worst`. Delayed scans retain their original header time, and the controller stops on stale header timestamps.

The final command path is `/cmd_vel_teleop` or `/cmd_vel_nav` → `twist_mux` → `/cmd_vel_in` → adaptive safety controller → `/cmd_vel`. This means keyboard teleoperation wins over navigation but cannot bypass emergency stop, sensor timeout, the footprint corridor, or braking margin:

`d_margin(v) = d_base + v(t_reaction + t_scan_latency) + v²/(2 a_brake)`.

Available policies are `fixed`, `distance_linear`, `density_only`, `distance_density`, `distance_braking`, and `distance_density_braking`. `fixed` is an ablation baseline only: emergency stop and sensor timeout remain enabled. `distance_density` is the normal default; braking variants are appropriate where stopping distance is the main constraint.

For a headless fault run, use `ros2 launch adaptive_speed_control full_demo.launch.py scenario:=3 headless:=true fault_profile:=delay_200ms`. Use `bag:=true` to record core safety topics under `bags/`; replay with `scripts/replay_bag.sh <bag>` and extract telemetry with `scripts/analyze_bag.py <bag>`. Summarise per-run CSVs with `scripts/analyze_results.py results`. Open `rqt_robot_monitor` to inspect `/diagnostics` during a GUI run.

No fault-study or ablation claims are reported in this README yet: a complete headless scenario matrix and a deliberate collision positive-control run still need to be executed.

### 7.1 Complete Autonomous Start-to-End Demonstration (Headless or GUI)

Run the full demonstration in headless mode (recommended for CI or remote environments):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=true
```

To run with graphical Gazebo simulation window and RViz visualization:
```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=false rviz:=true
```

### 7.2 Clearing the Zone 4 Barrier
When the robot encounters the barrier at $x=6.6\text{ m}$, it safely stops in `CRITICAL_PROXIMITY`. In a new terminal, clear the obstacle to trigger automatic recovery:
```bash
# Option A: Via ROS 2 Service Call
ros2 service call /clear_barrier std_srvs/srv/Trigger {}

# Option B: Via Python CLI Utility
ros2 run adaptive_speed_control clear_barrier
```
The robot immediately transitions to `PATH_CLEAR` and autonomously completes the corridor transit to the destination pad.

### 7.3 Manual Interactive Teleoperation
To manually drive the robot through the environment while safety regulation remains active:
```bash
# Terminal 1: Launch simulation
ros2 launch adaptive_speed_simulation spawn_robot.launch.py headless:=true

# Terminal 2: Launch controller & HUD
ros2 launch adaptive_speed_control adaptive_speed_control.launch.py

# Terminal 3: Launch interactive teleop UI
ros2 run adaptive_speed_control teleop_ui_node
```
Type commands in the format: `<linear_x> <angular_z> <duration_sec>` (e.g. `0.30 0.0 5.0`).

---

## 8. Verification & Test Suite

The repository incorporates 26 automated unit and integration tests executing under `pytest` and `ament_cmake`:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
```

### Test Coverage Breakdown
- **`test_speed_policy_comprehensive.py` (9 tests):** Boundary checks ($d=0, d=d_{safe}, d=\infty$), strict monotonic velocity attenuation across distance and clutter, hysteresis buffer validation, rate limiter acceleration ramp and instant step braking, NaN/Inf sensor beam filtering, and multi-sector geometric extraction.
- **`test_adaptive_speed.py` (10 tests):** Environmental zone speed clamping, reverse motion safety, and in-place rotation handling.
- **`test_navigator_state_machine.py` (3 tests):** Sequential waypoint progression, reactive flank obstacle nudging, and arrival detection.
- **`test_node_integration.py` (4 tests):** Live rclpy node integration, `/set_threshold` service validation, `/get_average_vel` rolling history, and timeout fail-safe halts.

**Result:** `26 passed in 0.42s (100% PASS)`.

---

## 9. Empirical Evaluation & Comparison

![Adaptive vs Fixed Benchmark](docs/figures/adaptive_vs_fixed_benchmark.png)

### Quantitative Comparison: Adaptive Speed Control vs Fixed Speed Baseline

| Metric | Fixed Speed Baseline | Adaptive Speed Controller | Impact / Advantage |
| :--- | :--- | :--- | :--- |
| **Zone 1 (Empty Corridor)** | $0.40\text{ m/s}$ | $0.40\text{ m/s}$ | Equal maximum transit throughput |
| **Zone 2 (Moderate Clutter)** | $0.40\text{ m/s}$ | $0.25 - 0.28\text{ m/s}$ | $35\%$ smoother navigation margin |
| **Zone 3 (Narrow Passage)** | $0.40\text{ m/s}$ (High Risk) | $0.10 - 0.15\text{ m/s}$ | **Zero collision risk**, optimal clearance |
| **Zone 4 (Emergency Barrier)** | Sudden hard crash / stop | Controlled deceleration to $0.00\text{ m/s}$ | Prevented structural mechanical impact |
| **Zone 5 (Warehouse Racks & Workers)** | High collision risk in 1.5m aisles | Adaptive speed + soft-stop for workers | Safe passage through narrow aisles & dynamic clutter |
| **Zone 6 (Hospital Reception & Lobby)** | Rigid motion near visitors | Smooth crowd regulation & pillar avoidance | Respects hospital comfort and pedestrian safety |
| **Automatic Resumption** | Requires manual restart | **Automatic resumption upon clearing** | Fully autonomous recovery |
| **Dynamic Acceleration** | Step jumps ($\infty$ jerk) | Smooth rate-limited ramp ($0.60\text{ m/s}^2$) | Reduced wheel slip and actuator wear |

---

## 10. Scenario Suite (Scenarios 1 through 6)

| Scenario | Environment World | Start Pose | Goal Pose | What It Tests | Expected Behavior |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **1: Open Corridor** | `my.sdf` (Zone 1) | $(-9.0, 0.0, 0^{\circ})$ | $(9.0, 0.0)$ | Maximum cruising speed in free, open space | Cruises at full speed ($0.40\text{ m/s}$), FREE zone, green trajectory trail |
| **2: Moderate Traffic** | `my.sdf` (Zone 2) | $(-4.0, 0.0, 0^{\circ})$ | $(9.0, 0.0)$ | Speed moderation under moderate obstacle density | Proactively decelerates to $0.25-0.28\text{ m/s}$, MODERATE zone (yellow trail) |
| **3: Cluttered Passage**| `my.sdf` (Zone 3) | $(0.5, 0.0, 0^{\circ})$ | $(9.0, 0.0)$ | Navigation through dense clutter and narrow passages | Drastically slows to $0.10-0.15\text{ m/s}$, DENSE zone (orange trail), maximum clearance |
| **4: Critical Barrier** | `my.sdf` (Zone 4) | $(5.0, 0.0, 0^{\circ})$ | $(9.0, 0.0)$ | Deterministic emergency braking and automatic resumption | Controlled stop at $0.45\text{ m}$ boundary (red); automatically resumes when barrier cleared |
| **5: Warehouse** | `scenario_5_warehouse.sdf` | $(-7.5, 0.0, 0^{\circ})$ | $(8.0, 0.0)$ | Realistic logistics: 4 shelf rows, pallets, cones, forklift, 3 dynamic workers | Navigates 90° corners, slows down inside narrow aisles, soft-stops for crossing workers |
| **6: Hospital Reception** | `scenario_6_hospital.sdf` | $(-7.5, 0.0, 0^{\circ})$ | $(7.5, 3.5)$ | Healthcare lobby: reception desk, waiting chairs, pillars, wheelchair, 2 dynamic visitors | Decelerates smoothly near desk and chairs, halts if pedestrian crosses, turns into ward corridor |

<p align="center">
  <img src="docs/figures/adaptive_scenario_5_trajectory.png" width="48%" alt="Scenario 5 Warehouse Trajectory">
  <img src="docs/figures/adaptive_scenario_5_speed_profile.png" width="48%" alt="Scenario 5 Speed Profile">
</p>

---

## 11. High-Visibility Speed Displays & Telemetry

The speed state is displayed in multiple concurrent modalities so it is impossible to miss:

1. **RViz Floating Speedometer**: A 3D text marker (`/visualization_speedometer`) floating directly above the robot displaying:
   `speed 0.18 m/s | limit 0.25 | zone DENSE`
   Color-coded: **Green** (fast $\ge 0.28\text{ m/s}$), **Yellow** (moderate $\ge 0.18\text{ m/s}$), **Orange** (slow $\ge 0.05\text{ m/s}$), **Red** (stopped $< 0.05\text{ m/s}$).
2. **Colour-Coded Trajectory Trail**: A persistent `visualization_msgs/Marker` `LINE_STRIP` (`/visualization_trajectory`) where each point is colored by the robot's actual Euclidean speed at that coordinate, complete with an on-screen legend marker (`/visualization_legend`).
3. **Ground Zone Ring**: A flat cylinder marker (`/visualization_zone_ring`) on the floor beneath the chassis colored by the safety zone:
   - Green: `FREE`
   - Yellow: `MODERATE`
   - Orange: `DENSE`
   - Red: `EMERGENCY / CRITICAL`
4. **Live Speed Dashboard GUI** (`speed_dashboard_node.py`):
   Launch with `dashboard:=true`. Displays:
   - Horizontal velocity gauge bar ($v_{act}$ vs $v_{max}$)
   - Real-time rolling plot of $v_{act}$, $v_{cmd}$, and $v_{limit}$ with colored background zone shading
   - Real-time numerical readout of safety threshold $d_{safe}$, front distance, and clutter density $\rho$
   - *Headless safety*: Automatically detects headless environments (missing `DISPLAY`) and runs in background logging mode without crashing.
5. **ANSI Color Terminal HUD**: `odometry_monitor_node` prints a 1-2 Hz terminal telemetry line with matching ANSI escape colors.
6. **RQt Plot Real-Time Inspection**:
   ```bash
   rqt_plot /robot_status/speed /robot_status/commanded_speed /robot_status/speed_limit
   ```

---

## 12. Quickstart & Demonstration Commands

### Scenario 5: Warehouse Logistics Demo
```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=5 autostart:=true
```
*(Optional with live dashboard: add `dashboard:=true`)*

### Scenario 6: Hospital Reception Demo
```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=6 autostart:=true
```

### Scenario 1-4 Corridor Demos
```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 autostart:=true
```

### Run All Benchmarks & Generate Plots
```bash
bash scripts/run_all_scenarios.sh
```

---

## 13. Viva & Technical Interview Questions & Answers

### Q1: Why decouple `/cmd_vel_in` and `/cmd_vel` using the Velocity Arbiter Pattern?
**Answer:** In multi-node robotic systems, if both the autonomous navigator and human teleoperation publish directly to `/cmd_vel`, message interleaving causes erratic actuator oscillation, missed safety stops, and race conditions. By routing all upstream commands through `/cmd_vel_in` and making the adaptive controller the *sole* publisher to `/cmd_vel`, safety constraints are guaranteed regardless of upstream software behavior.

### Q2: Why is obstacle density $\rho$ necessary in addition to closest distance $d$?
**Answer:** Proximity alone only informs the controller of the nearest single point. A robot passing a solitary post at $1.2\text{ m}$ in an open field can safely maintain speed, whereas passing through a narrow corridor lined on both sides with pillars at $1.2\text{ m}$ poses high risk due to odometry drift and lateral disturbances. Density $\rho$ quantifies environmental clutter and spatial confinement, allowing proactive speed moderation.

### Q3: How is spatial clutter density $\rho$ computed from a 2D LiDAR scan?
**Answer:** Over the front monitoring cone ($[-\theta_{front}, +\theta_{front}]$), the controller counts the number of valid laser returns that fall within a configurable radius $R_{density}$ ($1.30\text{ m}$) divided by the total number of valid returns in that sector: $\rho = N_{close} / N_{valid} \in [0.0, 1.0]$. Invalid readings (NaN, Inf, or below $range\_min$) are strictly filtered out beforehand.

### Q4: How does the system prevent chattering between behavioral states?
**Answer:** Through spatial hysteresis. When the robot halts in `CRITICAL_PROXIMITY` or `CONTROLLED_STOP` at $d \le 0.45\text{ m}$, it does not immediately resume the moment $d$ reaches $0.46\text{ m}$. Instead, it requires clearance beyond a hysteresis buffer ($d > d_{safe} + 0.20\text{ m} = 0.65\text{ m}$) before transitioning to `PATH_CLEAR` and resuming motion.

### Q5: Why is acceleration rate-limited while braking is immediate?
**Answer:** Physical differential-drive mobile robots experience wheel slip and internal strain when commanded with instantaneous step acceleration. Limiting positive acceleration ($a_{max} = 0.60\text{ m/s}^2$) ensures trajectory tracking and odometry fidelity. Conversely, safety requirements mandate instantaneous deceleration to zero when an obstacle encroaches within the safety threshold.

### Q6: How does the controller handle in-place rotation during an emergency stop?
**Answer:** When linear velocity is zero ($v_{req} = 0$), the robot is attempting pure rotation. The speed policy checks `abs(v_req) < 1e-6` and permits angular velocity $\omega$ (clamped to `max_angular_speed`), enabling the robot to pivot in place to find a safe heading away from an obstacle.

### Q7: What fail-safes are implemented for sensor loss or comms failure?
**Answer:** Watchdog timers monitor `last_scan_time` and `last_cmd_time`. If no `/scan` message arrives within $0.50\text{ s}$, the controller enters `SENSOR_TIMEOUT` and publishes zero velocity. If no command arrives on `/cmd_vel_in` within $0.50\text{ s}$, it enters `IDLE` and stops the robot.

### Q8: How does the Waypoint Navigator keep the robot centered in the corridor?
**Answer:** It computes cross-track error $e_y = y_{current} - y_{centerline}$. It calculates a lookahead target heading $\theta_{target} = \text{atan2}(-e_y, L_{lookahead})$ and applies proportional steering $\omega = k_p \cdot \text{normalize\_angle}(\theta_{target} - \theta_{yaw})$, guiding the robot back towards the centerline.

### Q9: What happens if the robot gets physically stuck?
**Answer:** `waypoint_navigator_node` tracks longitudinal progress ($x$). If the robot fails to advance by more than $0.15\text{ m}$ within $6.0\text{ s}$, it triggers `RECOVERY` mode: gently reversing for $1.5\text{ s}$, then rotating towards the sector (left vs right) that presents greater LiDAR clearance.

### Q10: How are Gazebo Harmonic and ROS 2 Jazzy bridged?
**Answer:** Using `ros_gz_bridge`'s `parameter_bridge` executable, bridging `/clock`, bidirectional `/cmd_vel`, `/odom`, `/joint_states`, and `/scan`. Simulation time is synchronized via `use_sim_time:=true`.

### Q11: How does the `/clear_barrier` service manipulate Gazebo simulation objects?
**Answer:** It invokes Gazebo Harmonic's `/world/empty/set_pose` service with request type `gz.msgs.Pose`, translating the entity `zone4_barrier` from $(x=6.6, y=0.0)$ to $(x=6.6, y=10.0)$, safely moving it out of the corridor.

### Q12: Why are pure functions extracted into `utils/speed_policy.py`?
**Answer:** Extracting core mathematics into stateless Python functions without `rclpy` dependencies permits exhaustive testing of boundary conditions, monotonicity, and mathematical invariants in sub-second pytest executions without overhead.

### Q13: What is the purpose of `RobotStatus.msg`?
**Answer:** It provides a standardized telemetry message on `/robot_status` aggregating true Euclidean speed, commanded speed, speed limit, active zone, distance to goal, cumulative distance travelled, and estimated time of arrival (ETA).

### Q14: How is ETA computed in `odometry_monitor_node`?
**Answer:** It maintains a rolling history of the last 10 actual velocity samples. If average velocity exceeds $0.05\text{ m/s}$, it calculates $\text{ETA} = d_{goal} / v_{avg}$; otherwise, it reports infinity (`999.0s`).

### Q15: How does the system support fixed-speed baseline benchmarking?
**Answer:** The controller exposes a boolean parameter `adaptive_enabled`. When set to `false` (e.g. via `baseline_fixed_speed.launch.py`), the controller bypasses distance and clutter attenuation, executing the nominal speed until an emergency stop is reached.

### Q16: How are dynamic moving obstacles (workers/pedestrians) simulated and detected in Gazebo Sim?
**Answer:** Dynamic obstacles are defined as standard Gazebo SDF models with full `<collision>` geometry (cylinders: radius $0.25\text{ m}$, height $1.7\text{ m}$). `moving_obstacle_node.py` smoothly translates them along configured 3D vectors at speeds of $0.2-0.5\text{ m/s}$ by calling the Gazebo `/world/<world_name>/set_pose` service bridged to ROS 2 via `ros_gz_interfaces/srv/SetEntityPose`. Because they possess genuine collision elements, Gazebo's Ray/GPU LiDAR sensor detects them continuously and accurately down to the millimeter without relying on synthetic mocks.

### Q17: Why are simple geometric primitives (boxes, cylinders, spheres) used instead of high-poly 3D meshes for LiDAR simulation?
**Answer:** Two fundamental reasons:
1. **Ray-tracing computational throughput**: Ray casting against analytic primitives (boxes, cylinders) runs in $O(1)$ ray-primitive intersection time, whereas high-poly meshes require bounding volume hierarchy (BVH) ray-triangle intersections that bottleneck simulation real-time factor (RTF). Simple geometry maintains RTF $\ge 1.00$ even in dense 20m x 14m worlds with 10+ obstacles and full LiDAR update rates.
2. **Offline reproducibility**: Fuel/internet-dependent meshes create fragility and break in air-gapped test runners or headless CI environments. Inherent geometric primitives render offline deterministically.

### Q18: How are the visual speed and zone display colors computed across RViz, the dashboard, and terminal readouts?
**Answer:** A unified four-tier color scheme maps instantaneous robot state across all visualization pipelines:
- **Green** ($\ge 0.28\text{ m/s}$, `FREE` zone): safe cruising at or near nominal velocity.
- **Yellow** ($0.18 \le v < 0.28\text{ m/s}$, `MODERATE` zone): proximity or clutter attenuation active.
- **Orange** ($0.05 \le v < 0.18\text{ m/s}$, `DENSE` zone): high obstacle density, crawl speed for tight maneuvering.
- **Red** ($< 0.05\text{ m/s}$, `CRITICAL / EMERGENCY` zone): immediate proximity stop at $d \le d_{safe}$.
This mapping is computed dynamically by `odometry_monitor_node` for RViz Markers and ANSI terminal output, and by `speed_dashboard_node` for matplotlib graphics.

### Q19: How does the robot distinguish between static clutter and dynamic obstacles, and what is the soft-stop waiting policy?
**Answer:** The navigator evaluates whether a sudden forward obstruction is temporal or static. When an obstacle blocks the path directly ahead ($d < 0.65\text{ m}$) where no lateral bypass is immediately available, the navigator enters a `WAITING` soft-stop state ($v=0, \omega=0$) for up to `wait_timeout` ($4.0\text{ s}$). If the dynamic obstacle walks past, clearance is restored and the robot resumes `DRIVE` without executing erratic maneuvers. If the blockage persists past the timeout (indicating a stationary barrier), it transitions to `RECOVERY` to back up and navigate around.

### Q20: Why are Gazebo Sim "actors" avoided for LiDAR safety validation, and how does `SetEntityPose` ensure collision detection fidelity?
**Answer:** In Gazebo Sim (Harmonic / Fortress), `<actor>` entities are designed primarily for visual skeletal animation and frequently lack physical physics/collision engine integration. Consequently, Ray/GPU LiDAR sensors do not cast rays against actor meshes, rendering them completely invisible to the robot's sensor pipeline. Defining them as standard `<model>` links and updating their positions via `/world/<world_name>/set_pose` ensures that both visual and collision bounding volumes are transformed in lockstep, guaranteeing 100% sensor detection fidelity.

---

## 14. Author & Acknowledgements

- **Author:** **Vikash** (B.Tech Computer Science & Engineering, IIIT Dharwad)
- **Original Assignment Template & Robot Model:** Endri Gjinaj (University of Genoa)
- **Institution:** Indian Institute of Information Technology Dharwad (IIIT Dharwad)

---

## 15. License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
