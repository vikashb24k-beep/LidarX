# Adaptive Speed Control in Cluttered Environments

## 1. Aim

Fixed-speed motion is inefficient in open space and unsafe near obstacles. This project implements a ROS 2 safety controller that receives a desired velocity, evaluates LiDAR data, and publishes a safer velocity for a differential-drive robot.

The controller must make the robot:

- move quickly when the path is open and sparse;
- slow down as an obstacle gets closer or clutter increases;
- stop for a critical obstacle, stale LiDAR data, or an expired upstream command.

## 2. ROS 2 data flow

```text
Waypoint navigator / teleoperation
              |
              | desired Twist: /cmd_vel_in
              v
Adaptive speed controller <---- /scan (LiDAR), /odom
              |
              | safe Twist: /cmd_vel
              v
       Gazebo differential-drive robot
```

The controller is the only component that publishes final motion commands to `/cmd_vel`.

Important topics:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | Obstacle ranges used for safety and clutter estimation. |
| `/cmd_vel_in` | `geometry_msgs/Twist` | Requested velocity from navigation or teleoperation. |
| `/cmd_vel` | `geometry_msgs/Twist` | Final, safety-limited velocity sent to Gazebo. |
| `/odom` | `nav_msgs/Odometry` | Robot position and measured velocity. |
| `/obstacle_info` | `adaptive_speed_interfaces/ObstacleInfo` | Controller telemetry. |
| `/safe_speed_limit` | `std_msgs/Float32` | Current speed limit before acceleration ramping. |

## 3. Concepts used

### LiDAR sector processing

Each LiDAR scan is filtered to discard invalid, infinite, and out-of-range readings. The controller extracts:

- nearest obstacle globally;
- nearest obstacle in the forward sector;
- left and right sector distances;
- front obstacle density, `rho`, the fraction of nearby valid rays.

### Footprint-aware swept corridor

The controller transforms LiDAR points from the sensor frame to the robot frame and checks a corridor in front of the robot. The corridor includes the robot width, a lateral safety margin, and a small turn-dependent offset. Therefore, side objects do not stop the robot unless they intrude into its future path.

### Obstacle clearance and braking margin

`d_clear` is the minimum clearance in the swept path. The dynamic safety margin is:

```text
d_margin = d_base + v * (t_reaction + t_scan_latency) + v^2 / (2 * a_brake)
```

It increases at higher speed because the robot needs more distance to react and brake.

### Clutter density

The controller uses front-path density to reduce speed in complex space. A density deadband ignores a small number of close rays from normal corridor walls or sensor noise, so a clear path can return to full speed.

```text
rho_effective = 0                              when rho <= rho_free
rho_effective = (rho - rho_free)/(1-rho_free)  otherwise
```

### Layered speed governor

The default `layered_governor` computes several caps and selects the most conservative safe result:

```text
safe_speed = min(requested_speed,
                 braking_cap,
                 comfort_cap,
                 lateral_cap,
                 clutter_cap,
                 turn_cap,
                 time_to_collision_cap,
                 goal_cap)
```

The main effects are:

- **braking cap:** the robot can stop before the base safety distance;
- **comfort cap:** speed smoothly rises from creep to requested speed as clearance improves;
- **clutter cap:** density reduces speed in crowded areas;
- **turn cap:** limits speed on high-curvature paths;
- **TTC cap:** restricts speed if closing time is too small;
- **goal cap:** reduces speed near the destination.

### Rate limiting and fail-safe behavior

Positive acceleration is limited for smooth movement. Deceleration is allowed immediately when safety requires it. The robot publishes zero velocity when LiDAR is stale, the upstream command expires, or clearance reaches the emergency threshold.

## 4. Current controller thresholds

Values below are from `src/adaptive_speed_control/config/adaptive_speed_params.yaml`.

| Parameter | Current value | Meaning |
| --- | ---: | --- |
| `threshold` | `0.45 m` | Emergency-stop base distance. |
| `slowdown_distance` | `2.00 m` | Distance at which slowdown behavior begins. |
| `density_radius` | `1.30 m` | Radius used to count nearby LiDAR rays. |
| `density_influence` | `0.60` | Strength of speed reduction caused by clutter. |
| `density_free_threshold` | `0.15` | Sparse density treated as open space. |
| `c_min` | `0.25` | Minimum density multiplier in non-layered policies. |
| `max_linear_speed` | `0.40 m/s` | Controller's configured linear-speed cap. |
| `max_angular_speed` | `1.00 rad/s` | Controller angular-speed cap. |
| `max_acceleration` | `0.60 m/s²` | Maximum positive acceleration. |
| `front_angle_deg` | `60°` | Half-angle of forward LiDAR sector. |
| `sensor_timeout` | `0.50 s` | Stop if a scan has not arrived in time. |
| `cmd_timeout` | `0.50 s` | Stop if desired commands stop arriving. |
| `control_rate` | `20 Hz` | Controller loop frequency. |
| `governor_v_creep` | `0.05 m/s` | Lowest nonzero governor command. |
| `governor_a_brake` | `0.60 m/s²` | Braking deceleration used for safety calculations. |
| `governor_ttc_min` | `1.00 s` | Minimum desired time-to-collision. |
| `governor_rho_gain` | `0.60` | Layered-governor clutter penalty. |

The waypoint navigator requests `nominal_speed: 0.40 m/s` in `src/adaptive_speed_control/config/navigator_params.yaml`. Increase this together with `max_linear_speed` if testing a higher nominal speed.

## 5. Simulation speed limits

There are multiple limits. The final speed is the lowest applicable limit:

1. Navigator requested speed: `0.40 m/s` by default.
2. Controller `max_linear_speed`: `0.40 m/s` by default.
3. Gazebo differential-drive `max_linear_velocity`: `0.5 m/s` in `src/adaptive_speed_simulation/urdf/mogi_bot.gazebo`.
4. Safety policy speed cap, based on clearance and clutter.

For a variation experiment, change the navigator request, controller cap, and Gazebo drive limit consistently. Do not use extreme values without increasing sensing range, slowdown distance, braking capability, and safety margins.

## 6. How to run the simulation

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run the complete autonomous simulation in headless mode:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=true autostart:=true
```

Run with the Gazebo GUI and RViz:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=false rviz:=true
```

Useful variations:

```bash
# Run a cluttered scenario.
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=3 headless:=true

# Compare with fixed-speed behavior; emergency stop remains active.
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=true adaptive_enabled:=false

# Test response to delayed LiDAR data.
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=3 headless:=true fault_profile:=delay_200ms
```

Supported fault profiles are `none`, `mild_noise`, `heavy_noise`, `dropout_10`, `delay_200ms`, and `combined_worst`.

## 7. Tuning experiments

Edit `src/adaptive_speed_control/config/adaptive_speed_params.yaml`, rebuild, and relaunch.

| Goal | Parameters to adjust | Safe direction |
| --- | --- | --- |
| Faster in clear space | `max_linear_speed`, navigator `nominal_speed` | Increase gradually together. |
| Faster with sparse side returns | `density_free_threshold` | Increase slightly, e.g. `0.15` to `0.20`. |
| More cautious in clutter | `density_influence`, `governor_rho_gain` | Increase. |
| Start slowing earlier | `slowdown_distance` | Increase. |
| Larger stand-off distance | `threshold` | Increase, keeping it below `slowdown_distance`. |
| Smooth faster pickup | `max_acceleration` | Increase cautiously. |

Never reduce `threshold`, `slowdown_distance`, or braking deceleration just to obtain a higher speed; those values are safety limits.

## 8. Validation

Run the automated checks after changing the policy:

```bash
python3 -m pytest src/adaptive_speed_control/test -q
```

The test suite covers LiDAR filtering, distance/density monotonicity, emergency stopping, recovery behavior, rate limiting, layered-governor limits, ROS node integration, and command/sensor timeouts.
