# QA test report

| Item | Command | Result | Notes |
|---|---|---|---|
| Package build | `colcon build --packages-select adaptive_speed_interfaces adaptive_speed_control --symlink-install` | PASS | Completed 2026-11-03. |
| Control tests | `ROS_LOG_DIR=/tmp/ros-log colcon test --packages-select adaptive_speed_control --event-handlers console_direct+` | PASS | 49 passed, 0 failed, 0 skipped. |
| Launch argument validation | `ROS_LOG_DIR=/tmp/ros-log ros2 launch adaptive_speed_control full_demo.launch.py --show-args` | PASS | Full and manual launch arguments loaded. |
| Headless launch | `timeout --signal=INT 35s ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=true` | FIXED | The simulation, robot, mux, controller, navigator, injector and monitor start. GPU lidar streams successfully headless. |
| Gazebo LiDAR source | `gz topic -e -t /scan_raw --msgtype gz.msgs.LaserScan --num 1` | PASS | Gazebo emits valid scan data with frame `scan_link`. |
| ROS LiDAR ingestion | `ros2 topic echo --once /scan_raw sensor_msgs/msg/LaserScan` during launch | FIXED | Received a 3.7 KB bridged LaserScan. The prior error was caused by the CPU lidar fallback, which advertised no Gazebo publisher. |
| Scenario 1 autonomous | `timeout --signal=INT 35s ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=true` | PASS | Controller left timeout, reached 0.40 m/s in clear space, and navigator logged `DESTINATION REACHED`. |
| Scenario matrix 2--6 | autonomous adaptive/fixed | NOT RUN | Remaining QA coverage. |
| GUI appearance | Gazebo/RViz/dashboard | NOT VERIFIABLE HEADLESS | Requires a desktop session. |

## Fixes made

1. Restored GPU LiDAR: it is the sensor that publishes successfully in this headless world; CPU lidar advertised no Gazebo publisher.
2. Configured the control stack with `use_sim_time:=true` so scan header age is compared against simulation time.
3. Remapped `twist_mux` output `/cmd_vel_out` to the controller input `/cmd_vel_in`.

## Remaining coverage

The previous `SENSOR_TIMEOUT` fault is resolved. The unexecuted scenario matrix, fault campaign, bags, and experiment runner remain unverified rather than failed.
