# Verified run guide

## 1. Build

On Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic installed:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2-lidar-safety-controller
colcon build --symlink-install
source install/setup.bash
```

## 2. Run unit tests

```bash
ROS_LOG_DIR=/tmp/ros-log colcon test --packages-select adaptive_speed_control --event-handlers console_direct+
colcon test-result --verbose
```

Verified result: 49 tests passed.

## 3. Autonomous demo [known issue]

```bash
export ROS_LOG_DIR=/tmp/ros-log
export GZ_LOG_PATH=/tmp/gz-log
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=true autostart:=true
```

This command starts the complete stack. In this verified environment it stops safely in `SENSOR_TIMEOUT` because the bridged LaserScan cannot be converted by rclpy; see `TEST_REPORT.md`. Do not treat this as a successful autonomous run until that issue is fixed.

Scenarios use `scenario:=1` through `scenario:=6`; select a baseline with `adaptive_enabled:=false`, and a fault profile with `fault_profile:=mild_noise` (also `heavy_noise`, `dropout_10`, `delay_200ms`, `combined_worst`).

## 4. Manual driving [manual check]

Terminal 1:

```bash
ros2 launch adaptive_speed_control manual_demo.launch.py scenario:=1
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run adaptive_speed_control keyboard_teleop_node
```

Keys: `w` forward, `s` reverse, `a` left, `d` right, `x`/Space stop. Commands flow through `/cmd_vel_teleop -> twist_mux -> /cmd_vel_in -> safety controller -> /cmd_vel`.

## 5. Bags and troubleshooting

Add `bag:=true` to the full-demo launch, replay with `scripts/replay_bag.sh <bag-directory>`, and extract telemetry with `scripts/analyze_bag.py <bag-directory>`.

If you see `SENSOR_TIMEOUT` immediately while `gz topic -e -t /scan_raw --msgtype gz.msgs.LaserScan --num 1` prints scans, you have the recorded bridge/type-support issue. Check that matching `ros_gz_bridge` and `ros_gz_interfaces` Jazzy packages are installed; a clean repair command on a fresh system is:

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces ros-jazzy-sensor-msgs
```

## Manual checklist

- [ ] In Gazebo/RViz, confirm START and GOAL markers and an unobstructed footprint path.
- [ ] Confirm `/scan_raw` and `/scan` stream continuously and the dashboard shows non-timeout zones.
- [ ] Confirm keyboard commands override navigation but cannot bypass emergency stop.
- [ ] Confirm collision and near-miss indicators change during a deliberate positive-control run.
