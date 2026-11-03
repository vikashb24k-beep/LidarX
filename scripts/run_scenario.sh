#!/usr/bin/env bash
set -e

SCENARIO="${1:-1}"
MODE="${2:-adaptive}" # adaptive or fixed
DURATION="${3:-45}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CSV_FILE="${WS_DIR}/results/${MODE}_scenario_${SCENARIO}.csv"

echo "================================================================"
echo " Running Scenario ${SCENARIO} [Mode: ${MODE}] for ${DURATION}s"
echo " Output CSV: ${CSV_FILE}"
echo "================================================================"

# Source ROS environment
source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"

# Cleanup any previous processes
killall -9 gz-sim-server ruby gz 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
sleep 2

ADAPTIVE_FLAG="true"
if [ "$MODE" = "fixed" ]; then
    ADAPTIVE_FLAG="false"
fi

# Run demo with timeout
timeout --preserve-status "${DURATION}s" ros2 launch adaptive_speed_control full_demo.launch.py \
    scenario:="${SCENARIO}" \
    headless:=true \
    rviz:=false \
    autostart:=true \
    log_metrics:=true \
    csv_filename:="${CSV_FILE}" || true

echo ">> Run complete. Cleaning up simulation processes..."
killall -9 gz-sim-server ruby gz 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
sleep 1

if [ -f "${CSV_FILE}" ]; then
    LINES=$(wc -l < "${CSV_FILE}")
    echo ">> Successfully generated ${CSV_FILE} (${LINES} samples recorded)."
else
    echo ">> [WARNING] CSV file was not generated."
fi
