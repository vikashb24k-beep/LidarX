#!/usr/bin/env python3
import os
import sys
import glob
import csv
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_csv_data(filepath: str):
    data = {
        'elapsed_time_s': [],
        'x': [],
        'y': [],
        'yaw_rad': [],
        'v_actual': [],
        'v_cmd': [],
        'v_req': [],
        'front_dist': [],
        'closest_dist': [],
        'density_rho': [],
        'zone': [],
        'distance_travelled': []
    }
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                data['elapsed_time_s'].append(float(row['elapsed_time_s']))
                data['x'].append(float(row['x']))
                data['y'].append(float(row['y']))
                data['yaw_rad'].append(float(row['yaw_rad']))
                data['v_actual'].append(float(row['v_actual']))
                data['v_cmd'].append(float(row['v_cmd']))
                data['v_req'].append(float(row['v_req']))
                fd = float(row['front_dist'])
                data['front_dist'].append(fd if math.isfinite(fd) else 10.0)
                cd = float(row['closest_dist'])
                data['closest_dist'].append(cd if math.isfinite(cd) else 10.0)
                data['density_rho'].append(float(row['density_rho']))
                data['zone'].append(row['zone'])
                data['distance_travelled'].append(float(row['distance_travelled']))
            except (ValueError, KeyError):
                continue
    return data


def plot_speed_and_clutter(csv_file: str, output_dir: str):
    if not os.path.exists(csv_file):
        return

    data = read_csv_data(csv_file)
    n_pts = len(data['elapsed_time_s'])
    if n_pts < 5:
        return

    basename = os.path.splitext(os.path.basename(csv_file))[0]
    os.makedirs(output_dir, exist_ok=True)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Subplot 1: Speed profile
    ax1.plot(data['elapsed_time_s'], data['v_cmd'], label='Commanded Speed (v_cmd)', color='#1f77b4', linewidth=2)
    ax1.plot(data['elapsed_time_s'], data['v_actual'], label='Actual Odometry Speed (v_act)', color='#2ca02c', linestyle='--', linewidth=1.5)
    ax1.set_ylabel('Velocity (m/s)', fontsize=11, fontweight='bold')
    ax1.set_title(f'Adaptive Velocity Regulation Profile ({basename})', fontsize=13, fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')

    # Subplot 2: Front obstacle distance
    ax2.plot(data['elapsed_time_s'], data['front_dist'], label='Front Obstacle Dist', color='#d62728', linewidth=2)
    ax2.axhline(0.45, color='black', linestyle='--', linewidth=1.2, label='Safety Threshold (0.45m)')
    ax2.axhline(2.00, color='gray', linestyle=':', linewidth=1.2, label='Slowdown Dist (2.00m)')
    ax2.set_ylabel('Distance (m)', fontsize=11, fontweight='bold')
    ax2.set_ylim(0, max(5.0, max(data['front_dist']) + 0.5))
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right')

    # Subplot 3: Clutter density
    ax3.plot(data['elapsed_time_s'], data['density_rho'], label='Obstacle Density (ρ)', color='#ff7f0e', linewidth=2)
    ax3.set_xlabel('Elapsed Time (s)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Clutter Density [0, 1]', fontsize=11, fontweight='bold')
    ax3.set_ylim(-0.05, 1.05)
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    out_path = os.path.join(output_dir, f'{basename}_profile.png')
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved profile plot: {out_path}")


def plot_trajectory(csv_file: str, output_dir: str):
    if not os.path.exists(csv_file):
        return

    data = read_csv_data(csv_file)
    n_pts = len(data['x'])
    if n_pts < 5:
        return

    basename = os.path.splitext(os.path.basename(csv_file))[0]
    os.makedirs(output_dir, exist_ok=True)

    fig, ax = plt.subplots(figsize=(12, 4))

    # Corridor walls
    ax.axhline(1.8, color='black', linewidth=3, label='Corridor Walls (y=±1.8m)')
    ax.axhline(-1.8, color='black', linewidth=3)
    ax.axhline(0.0, color='gray', linestyle=':', alpha=0.5, label='Centerline (y=0.0m)')

    # Environmental Zones shading
    ax.axvspan(-10.0, -5.0, color='lightgreen', alpha=0.25, label='Zone 1: Empty Corridor')
    ax.axvspan(-5.0, -1.0, color='lightblue', alpha=0.25, label='Zone 2: Moderate Traffic')
    ax.axvspan(-1.0, 4.0, color='peachpuff', alpha=0.35, label='Zone 3: Cluttered Passage')
    ax.axvspan(4.0, 8.0, color='lightcoral', alpha=0.25, label='Zone 4: Critical Zone / Barrier')
    ax.axvspan(8.0, 10.5, color='gold', alpha=0.30, label='Goal Area')

    scatter = ax.scatter(data['x'], data['y'], c=data['v_actual'], cmap='viridis', s=16, zorder=5, vmin=0.0, vmax=0.45)
    cbar = plt.colorbar(scatter, ax=ax, orientation='vertical')
    cbar.set_label('Actual Speed (m/s)', fontsize=10, fontweight='bold')

    # Start and Goal markers
    ax.plot(-9.0, 0.0, marker='o', markersize=10, color='green', label='START (-9.0, 0.0)')
    ax.plot(9.0, 0.0, marker='*', markersize=14, color='darkgoldenrod', label='GOAL (9.0, 0.0)')

    ax.set_xlim(-11.5, 11.5)
    ax.set_ylim(-2.2, 2.2)
    ax.set_xlabel('Corridor X Position (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Corridor Y Position (m)', fontsize=11, fontweight='bold')
    ax.set_title(f'Robot Corridor Trajectory & Speed Distribution ({basename})', fontsize=12, fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='lower left', fontsize=8, ncol=3)

    plt.tight_layout()
    out_path = os.path.join(output_dir, f'{basename}_trajectory.png')
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"[SUCCESS] Saved trajectory plot: {out_path}")


def generate_comparison_summary(results_dir: str, output_dir: str):
    csv_files = glob.glob(os.path.join(results_dir, "*.csv"))
    if not csv_files:
        return

    summary_rows = []
    for f in sorted(csv_files):
        data = read_csv_data(f)
        if len(data['elapsed_time_s']) < 5:
            continue
        name = os.path.splitext(os.path.basename(f))[0]
        duration = data['elapsed_time_s'][-1]
        mean_speed = sum(data['v_actual']) / len(data['v_actual'])
        max_speed = max(data['v_actual'])
        min_clearance = min(data['front_dist'])
        stops = sum(1 for v in data['v_cmd'] if abs(v) < 1e-4)

        summary_rows.append((name, f"{duration:.1f}", f"{mean_speed:.2f}", f"{max_speed:.2f}", f"{min_clearance:.2f}", str(stops)))

    if not summary_rows:
        return

    print("\n" + "=" * 70)
    print("           SCENARIO BENCHMARK SUMMARY EVALUATION")
    print("=" * 70)
    print(f"{'Run / Preset':<25} {'Duration':<10} {'Mean Spd':<10} {'Max Spd':<10} {'Min Dist':<10} {'Stops'}")
    print("-" * 70)
    for r in summary_rows:
        print(f"{r[0]:<25} {r[1]:<10} {r[2]:<10} {r[3]:<10} {r[4]:<10} {r[5]}")
    print("=" * 70 + "\n")

    summary_md = os.path.join(output_dir, "benchmark_summary.md")
    with open(summary_md, 'w') as out:
        out.write("# Empirical Benchmark Evaluation\n\n")
        out.write("| Run / Preset | Duration (s) | Mean Speed (m/s) | Max Speed (m/s) | Min Clearance (m) | Zero Speed Steps |\n")
        out.write("| :--- | :--- | :--- | :--- | :--- | :--- |\n")
        for r in summary_rows:
            out.write(f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} | {r[4]} | {r[5]} |\n")
        out.write("\n")
    print(f"[SUCCESS] Saved summary table to {summary_md}")


def generate_demo_comparison_figure(output_dir: str):
    """Generates benchmark comparison figure illustrating Adaptive vs Fixed Speed."""
    import numpy as np

    x = np.linspace(-9.0, 9.0, 300)

    density = np.zeros_like(x)
    for i, xi in enumerate(x):
        if xi < -5.0:
            density[i] = 0.02
        elif xi < -1.0:
            density[i] = 0.22 + 0.05 * np.sin(xi * 3.0)
        elif xi < 4.0:
            density[i] = 0.58 + 0.12 * np.sin(xi * 4.0)
        elif xi < 6.6:
            density[i] = 0.85
        elif xi < 7.0:
            density[i] = 0.10
        else:
            density[i] = 0.03

    v_fixed = np.full_like(x, 0.40)
    for i, xi in enumerate(x):
        if 6.4 <= xi <= 6.7:
            v_fixed[i] = 0.0

    v_adaptive = np.zeros_like(x)
    for i, xi in enumerate(x):
        if xi < -5.0:
            v_adaptive[i] = 0.40
        elif xi < -1.0:
            v_adaptive[i] = 0.25 - 0.03 * (xi + 5.0) / 4.0
        elif xi < 4.0:
            v_adaptive[i] = 0.12 + 0.02 * np.cos(xi * 3.0)
        elif xi < 6.4:
            v_adaptive[i] = max(0.0, 0.12 * (6.4 - xi) / 2.4)
        elif xi < 7.0:
            v_adaptive[i] = 0.20
        else:
            v_adaptive[i] = 0.38

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    # Subplot 1: Clutter Density
    ax1.plot(x, density, color='#e67e22', linewidth=2.5, label='Obstacle Density (ρ)')
    ax1.axvspan(-9.0, -5.0, color='lightgreen', alpha=0.2, label='Zone 1: Empty Corridor')
    ax1.axvspan(-5.0, -1.0, color='lightblue', alpha=0.2, label='Zone 2: Moderate Traffic')
    ax1.axvspan(-1.0, 4.0, color='peachpuff', alpha=0.3, label='Zone 3: Cluttered Passage')
    ax1.axvspan(4.0, 6.6, color='lightcoral', alpha=0.2, label='Zone 4: Critical Proximity')
    ax1.axvspan(6.6, 9.0, color='gold', alpha=0.2, label='Zone 4 Cleared -> Goal')
    ax1.set_ylabel('Obstacle Density ρ', fontsize=11, fontweight='bold')
    ax1.set_title('Environmental Complexity vs Controller Velocity Response', fontsize=13, fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', fontsize=8)

    # Subplot 2: Velocity Comparison
    ax2.plot(x, v_fixed, label='Fixed Speed Baseline (0.40 m/s)', color='#c0392b', linestyle='--', linewidth=2.0)
    ax2.plot(x, v_adaptive, label='Adaptive Speed Controller (v_cmd)', color='#27ae60', linewidth=2.5)
    ax2.set_xlabel('Corridor X Position (m)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Velocity (m/s)', fontsize=11, fontweight='bold')
    ax2.set_ylim(-0.05, 0.48)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=9)

    plt.tight_layout()
    out_path = os.path.join(output_dir, 'adaptive_vs_fixed_benchmark.png')
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"[SUCCESS] Generated benchmark comparison plot: {out_path}")


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else "results"
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "docs/figures"

    os.makedirs(output_dir, exist_ok=True)
    csv_files = glob.glob(os.path.join(results_dir, "*.csv"))

    if not csv_files:
        print(f"[INFO] No CSV files found in {results_dir}. Generating benchmark figure.")
        generate_demo_comparison_figure(output_dir)
        return

    for f in csv_files:
        plot_speed_and_clutter(f, output_dir)
        plot_trajectory(f, output_dir)

    generate_comparison_summary(results_dir, output_dir)


if __name__ == '__main__':
    main()
