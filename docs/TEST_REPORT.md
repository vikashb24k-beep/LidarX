# QA test report

| Item | Command | Result | Notes |
|---|---|---|---|
| Package build | `colcon build --packages-select adaptive_speed_interfaces adaptive_speed_control --symlink-install` | PASS | Completed 2026-11-03. |
| Control tests | `ROS_LOG_DIR=/tmp/ros-log colcon test --packages-select adaptive_speed_control --event-handlers console_direct+` | PASS | 49 passed, 0 failed, 0 skipped. |
| Launch argument validation | `ROS_LOG_DIR=/tmp/ros-log ros2 launch adaptive_speed_control full_demo.launch.py --show-args` | PASS | Full and manual launch arguments loaded. |
| Headless launch | `timeout --signal=INT 15s ros2 launch adaptive_speed_control full_demo.launch.py scenario:=1 headless:=true` | FIXED / KNOWN ISSUE | The simulation, robot, mux, controller, navigator, injector and monitor start. CPU lidar was selected for headless use. |
| Gazebo LiDAR source | `gz topic -e -t /scan_raw --msgtype gz.msgs.LaserScan --num 1` | PASS | Gazebo emits valid scan data with frame `scan_link`. |
| ROS LiDAR ingestion | same headless launch | KNOWN ISSUE | `scan_fault_injector_node` fails taking a bridged LaserScan: `RuntimeError: Unable to convert call argument '0' to Python object`, seen in `/tmp/simtime_qa.log`. Controller correctly remains `SENSOR_TIMEOUT`. |
| Scenario matrix 1--6 | autonomous adaptive/fixed | NOT RUN | Blocked by ROS LaserScan conversion issue; no success/collision claims made. |
| GUI appearance | Gazebo/RViz/dashboard | NOT VERIFIABLE HEADLESS | Requires a desktop session. |

## Fixes made

1. Switched the model to CPU `lidar` from `gpu_lidar` for headless compatibility.
2. Configured the control stack with `use_sim_time:=true` so scan header age is compared against simulation time.
3. Matched the injector ingress QoS to the reliable Gazebo bridge offer. This did not bypass the Python conversion exception.

## Known issue and evidence

The native Gazebo command proves `/scan_raw` is populated. The ROS bridge creates the advertised `sensor_msgs/msg/LaserScan` publisher, but a Python rclpy subscriber fails when it takes its first sample. This is an installed `ros_gz_bridge`/Python LaserScan type-support incompatibility, not a safety-controller bypass. The controller's zero-velocity `SENSOR_TIMEOUT` response was observed repeatedly. Until it is corrected, end-to-end navigation, fault campaigns, bag analysis, and experiment runs are not verifiable.
