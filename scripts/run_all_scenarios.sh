#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "================================================================"
echo " Running Automated Benchmark Scenarios for Adaptive Speed Control"
echo "================================================================"

mkdir -p "${WS_DIR}/results"
mkdir -p "${WS_DIR}/docs/figures"

# Run Scenario 1 (Open corridor) for 20s
"${SCRIPT_DIR}/run_scenario.sh" 1 adaptive 20

# Run Scenario 2 (Moderate traffic) for 20s
"${SCRIPT_DIR}/run_scenario.sh" 2 adaptive 20

# Run Scenario 3 (Cluttered passage) for 25s
"${SCRIPT_DIR}/run_scenario.sh" 3 adaptive 25

# Generate plots and summary report
python3 "${SCRIPT_DIR}/plot_results.py" "${WS_DIR}/results" "${WS_DIR}/docs/figures"

echo "================================================================"
echo " All scenarios completed. Figures saved to docs/figures/"
echo "================================================================"
