# Start Guide

Run every command from the repository root:

```bash
cd /home/vhack/vikash/ros2-lidar-safety-controller
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If you changed source code, rebuild once before launching:

```bash
colcon build --symlink-install
source install/setup.bash
```

Use one scenario at a time. Stop the current simulation with `Ctrl+C` before starting another.

If a newly selected scenario still shows the previous map, an older Gazebo simulation is running. Return to the terminal that started it, press `Ctrl+C`, close its Gazebo window, and then start the new scenario. Do not start a second scenario while the first one is still running.

## Scenario 1 — Open corridor

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 autostart:=true
```

## Scenario 2 — Moderate corridor

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=2 autostart:=true
```

## Scenario 3 — Cluttered corridor

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=3 autostart:=true
```

## Scenario 4 — Barrier corridor

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=4 autostart:=true
```

## Scenario 5 — Warehouse

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=5 autostart:=true
```

## Scenario 6 — Hospital

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=6 autostart:=true
```

Scenarios 1–4 share the same corridor world and differ by their starting position. Scenarios 5 and 6 load separate warehouse and hospital worlds with their own waypoint routes.

## Run without the Gazebo GUI

Add `headless:=true` to any scenario command:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=5 headless:=true autostart:=true
```

## Keyboard robot control

Keep the scenario launch running in Terminal 1. In Terminal 2, source ROS 2 and the workspace, then start teleoperation:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run adaptive_speed_control keyboard_teleop_node
```

Controls: `w` forward, `s` reverse, `a` left, `d` right, `x` or Space stop, `+`/`-` adjust speed, and `t` switches manual/autonomous mode. Press `h` for the full key list.

## Dashboards

### Graphical speed dashboard

Add `dashboard:=true` to the scenario launch:

```bash
ros2 launch adaptive_speed_control full_demo.launch.py scenario:=5 autostart:=true dashboard:=true
```

### Terminal manual-driving dashboard

Keep the scenario launch running. In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run adaptive_speed_control manual_dashboard_node
```

The terminal dashboard reports requested, safe, and actual speed; nearest obstacle distances; the safety zone; heading; and distance travelled.
