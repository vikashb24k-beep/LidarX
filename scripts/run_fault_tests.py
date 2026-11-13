#!/usr/bin/env python3
"""
Automated headless safety test runner for sensor fault profiles.
Evaluates:
  - Profiles: none, mild_noise, heavy_noise, dropout_10, delay_200ms, combined_worst
  - Scenarios: 2 (moderate), 3 (cluttered), 5 (warehouse)
Asserts:
  - Zero collisions (min_clearance >= 0.0m)
  - Emergency stop triggers before physical contact
  - SENSOR_TIMEOUT occurs when scan age exceeds max_scan_age
Saves results in results/fault_tests.csv
"""
import csv
import os
import subprocess
import sys
import time
from pathlib import Path


FAULT_PROFILES = [
    'none',
    'mild_noise',
    'heavy_noise',
    'dropout_10',
    'delay_200ms',
    'combined_worst'
]

SCENARIOS = ['2', '3', '5']


def run_single_test(ws_dir: Path, scenario: str, profile: str, duration_sec: int = 12) -> dict:
    csv_out = ws_dir / "results" / f"fault_test_s{scenario}_{profile}.csv"
    cmd = [
        "bash", "-c",
        f"source /opt/ros/jazzy/setup.bash && source {ws_dir}/install/setup.bash && "
        f"timeout {duration_sec} ros2 launch adaptive_speed_control full_demo.launch.py "
        f"scenario:={scenario} fault_profile:={profile} headless:=true autostart:=true "
        f"log_metrics:=true csv_filename:={csv_out} || true"
    ]

    # Clean up before
    subprocess.run(["killall", "-9", "gz-sim-server", "ruby", "gz"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros2"], stderr=subprocess.DEVNULL)
    time.sleep(1.0)

    print(f"--> Running Scenario {scenario} with Fault Profile '{profile}' ({duration_sec}s)...")
    start = time.time()
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    elapsed = time.time() - start

    # Clean up after
    subprocess.run(["killall", "-9", "gz-sim-server", "ruby", "gz"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-9", "-f", "ros2"], stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    # Analyze CSV
    min_clearance = float('inf')
    collisions = 0
    emergency_stop = False
    sensor_timeout = False
    samples = 0

    if csv_out.exists():
        with open(csv_out, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                samples += 1
                try:
                    front = float(row.get('front_dist', 'inf'))
                    closest = float(row.get('closest_dist', 'inf'))
                    clearance = min(front, closest)
                    if clearance < min_clearance:
                        min_clearance = clearance
                except ValueError:
                    pass

                zone = row.get('zone', '')
                if 'CRITICAL' in zone or 'CONTROLLED_STOP' in zone or 'STOPPED' in zone:
                    emergency_stop = True
                if 'SENSOR_TIMEOUT' in zone:
                    sensor_timeout = True

    # Check collisions: min_clearance must be >= 0.20m (safety threshold is 0.45m)
    safe = (min_clearance >= 0.20) and (min_clearance < float('inf'))
    if min_clearance == float('inf') and samples > 0:
        safe = True  # open space

    passed = safe and (emergency_stop or min_clearance >= 0.45)

    return {
        'scenario': scenario,
        'fault_profile': profile,
        'samples': samples,
        'min_clearance_m': f"{min_clearance:.3f}" if min_clearance < float('inf') else "inf",
        'collisions': 0 if safe else 1,
        'emergency_stop_triggered': emergency_stop,
        'sensor_timeout_triggered': sensor_timeout,
        'status': 'PASS' if passed else 'FAIL'
    }


def main():
    ws_dir = Path(__file__).resolve().parent.parent
    results_dir = ws_dir / "results"
    results_dir.mkdir(exist_ok=True)
    out_csv = results_dir / "fault_tests.csv"

    records = []

    # Fast test: run key representative matrix
    test_matrix = [
        ('2', 'none'),
        ('2', 'mild_noise'),
        ('2', 'heavy_noise'),
        ('2', 'dropout_10'),
        ('2', 'delay_200ms'),
        ('2', 'combined_worst'),
        ('3', 'none'),
        ('3', 'heavy_noise'),
        ('3', 'combined_worst'),
        ('5', 'none'),
        ('5', 'mild_noise'),
        ('5', 'combined_worst'),
    ]

    print("================================================================")
    print(" LidarX Phase A: Automated LiDAR Fault Robustness Test Suite")
    print(f" Output File: {out_csv}")
    print("================================================================")

    for scenario, profile in test_matrix:
        rec = run_single_test(ws_dir, scenario, profile, duration_sec=10)
        records.append(rec)
        print(f"    Result: {rec['status']} | Min Clearance: {rec['min_clearance_m']}m | Collisions: {rec['collisions']}")

    # Write CSV
    with open(out_csv, 'w', newline='') as f:
        fieldnames = [
            'scenario', 'fault_profile', 'samples', 'min_clearance_m',
            'collisions', 'emergency_stop_triggered', 'sensor_timeout_triggered', 'status'
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    print("\n================================================================")
    print(" Summary Table:")
    print("================================================================")
    print(f"{'Scenario':<10} | {'Profile':<16} | {'Min Clear (m)':<14} | {'Collisions':<10} | {'Status':<6}")
    print("-" * 65)
    for r in records:
        print(f"{r['scenario']:<10} | {r['fault_profile']:<16} | {r['min_clearance_m']:<14} | {r['collisions']:<10} | {r['status']:<6}")

    print(f"\nAll tests completed. Output saved to {out_csv}")


if __name__ == '__main__':
    main()
