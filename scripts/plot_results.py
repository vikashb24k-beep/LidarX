#!/usr/bin/env python3
import os
import sys
import glob
import csv
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches


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


def draw_warehouse_map(ax):
    """Draws obstacle geometry for Scenario 5: Warehouse."""
    ax.add_patch(patches.Rectangle((-10, -7), 20, 14, fill=False, edgecolor='#333333', linewidth=3, label='Walls'))
    # Shelf rows
    ax.add_patch(patches.Rectangle((-3.5, 3.1), 8.0, 1.0, facecolor='#d95f02', edgecolor='#7a3400', alpha=0.75, label='Shelf Racks'))
    ax.add_patch(patches.Rectangle((-3.5, 0.4), 8.0, 1.0, facecolor='#d95f02', edgecolor='#7a3400', alpha=0.75))
    ax.add_patch(patches.Rectangle((-3.5, -2.3), 8.0, 1.0, facecolor='#d95f02', edgecolor='#7a3400', alpha=0.75))
    ax.add_patch(patches.Rectangle((-3.5, -5.0), 8.0, 1.0, facecolor='#d95f02', edgecolor='#7a3400', alpha=0.75))
    # Pallet stacks & boxes
    ax.add_patch(patches.Rectangle((-5.6, -3.7), 1.2, 1.0, facecolor='#8c510a', edgecolor='black', alpha=0.8, label='Pallets/Clutter'))
    ax.add_patch(patches.Rectangle((5.35, -2.65), 0.9, 0.9, facecolor='#8c510a', edgecolor='black', alpha=0.8))
    # Forklift
    ax.add_patch(patches.Rectangle((5.5, 3.1), 1.8, 1.1, facecolor='#e6ab02', edgecolor='black', alpha=0.9, label='Parked Forklift'))
    # Workers (Static & Dynamic)
    worker_coords = [(-7.5, 2.0), (0.0, 4.8), (-1.5, -2.8), (5.2, 4.5), (7.2, -2.5), (-5.5, 0.0), (1.5, 2.25), (-6.0, -1.0)]
    for i, (wx, wy) in enumerate(worker_coords):
        lbl = 'Workers / Moving' if i == 0 else None
        col = '#e41a1c' if i >= 5 else '#377eb8'
        ax.add_patch(patches.Circle((wx, wy), 0.25, facecolor=col, edgecolor='black', alpha=0.85, label=lbl))

    ax.set_xlim(-11, 11)
    ax.set_ylim(-8, 8)


def draw_hospital_map(ax):
    """Draws obstacle geometry for Scenario 6: Hospital Reception."""
    ax.add_patch(patches.Rectangle((-9, -6), 18, 12, fill=False, edgecolor='#333333', linewidth=3, label='Walls'))
    # Corridor partition walls
    ax.add_patch(patches.Rectangle((3.875, -6.0), 0.25, 7.5, facecolor='#999999', edgecolor='black', label='Partitions'))
    ax.add_patch(patches.Rectangle((4.0, 1.675), 5.0, 0.25, facecolor='#999999', edgecolor='black'))
    # Reception Desk
    ax.add_patch(patches.Rectangle((-0.2, 1.6), 0.8, 3.2, facecolor='#b3de69', edgecolor='#4d9221', alpha=0.8, label='Reception Desk'))
    ax.add_patch(patches.Rectangle((-1.7, 4.0), 1.6, 0.8, facecolor='#b3de69', edgecolor='#4d9221', alpha=0.8))
    # Waiting Chairs (3 Rows)
    ax.add_patch(patches.Rectangle((-3.5, -2.325), 2.6, 0.65, facecolor='#80b1d3', edgecolor='black', alpha=0.8, label='Waiting Chairs'))
    ax.add_patch(patches.Rectangle((-3.5, -3.525), 2.6, 0.65, facecolor='#80b1d3', edgecolor='black', alpha=0.8))
    ax.add_patch(patches.Rectangle((-3.5, -4.725), 2.6, 0.65, facecolor='#80b1d3', edgecolor='black', alpha=0.8))
    # Pillars
    ax.add_patch(patches.Circle((1.8, -0.8), 0.35, facecolor='#666666', edgecolor='black', label='Pillars'))
    ax.add_patch(patches.Circle((-5.0, 2.8), 0.35, facecolor='#666666', edgecolor='black'))
    # Wheelchair
    ax.add_patch(patches.Rectangle((-5.375, -3.825), 0.75, 0.65, facecolor='#bc80bd', edgecolor='black', label='Wheelchair'))
    # People (Staff & Visitors)
    people_coords = [(0.8, 2.5), (0.8, 3.8), (-0.4, 2.4), (-1.4, -2.0), (-2.2, -3.2), (-6.8, 2.5), (0.0, 0.5), (-4.0, 0.0)]
    for i, (px, py) in enumerate(people_coords):
        lbl = 'People / Visitors' if i == 0 else None
        col = '#4daf4a' if i < 2 else ('#e41a1c' if i >= 6 else '#984ea3')
        ax.add_patch(patches.Circle((px, py), 0.25, facecolor=col, edgecolor='black', alpha=0.85, label=lbl))

    ax.set_xlim(-10, 10)
    ax.set_ylim(-7, 7)


def plot_trajectory(csv_file: str, output_dir: str):
    if not os.path.exists(csv_file):
        return

    data = read_csv_data(csv_file)
    n_pts = len(data['x'])
    if n_pts < 5:
        return

    basename = os.path.splitext(os.path.basename(csv_file))[0]
    os.makedirs(output_dir, exist_ok=True)

    is_scenario_5 = 'scenario_5' in basename or (min(data['y']) < -2.0 and max(data['y']) > 1.5)
    is_scenario_6 = 'scenario_6' in basename or (max(data['y']) > 3.0)

    if is_scenario_5:
        fig, (ax_map, ax_speed) = plt.subplots(1, 2, figsize=(15, 6), gridspec_kw={'width_ratios': [1.3, 1]})
        draw_warehouse_map(ax_map)

        scatter = ax_map.scatter(data['x'], data['y'], c=data['v_actual'], cmap='viridis', s=22, zorder=6, vmin=0.0, vmax=0.45)
        cbar = plt.colorbar(scatter, ax=ax_map, orientation='horizontal', pad=0.08, shrink=0.7)
        cbar.set_label('Actual Speed (m/s)', fontsize=10, fontweight='bold')

        ax_map.plot(data['x'][0], data['y'][0], marker='o', markersize=10, color='green', label='START', zorder=7)
        ax_map.plot(data['x'][-1], data['y'][-1], marker='*', markersize=14, color='gold', label='CURRENT / GOAL', zorder=7)
        ax_map.set_xlabel('Warehouse X (m)', fontsize=10, fontweight='bold')
        ax_map.set_ylabel('Warehouse Y (m)', fontsize=10, fontweight='bold')
        ax_map.set_title(f'Scenario 5: Warehouse Trajectory ({basename})', fontsize=11, fontweight='bold')
        ax_map.legend(loc='upper right', fontsize=7, ncol=2)
        ax_map.grid(True, linestyle=':', alpha=0.4)

        t = data['elapsed_time_s']
        v = data['v_actual']
        ax_speed.plot(t, data['v_cmd'], label='Commanded (v_cmd)', color='#1f77b4', lw=2)
        ax_speed.plot(t, v, label='Actual (v_act)', color='#2ca02c', lw=2, ls='--')
        ax_speed.set_xlabel('Elapsed Time (s)', fontsize=10, fontweight='bold')
        ax_speed.set_ylabel('Speed (m/s)', fontsize=10, fontweight='bold')
        ax_speed.set_title(f'Velocity Profile & Obstacle Slowdowns', fontsize=11, fontweight='bold')
        ax_speed.grid(True, linestyle=':', alpha=0.5)
        ax_speed.legend(loc='upper right', fontsize=8)

        for i in range(len(t) - 1):
            z = data['zone'][i].upper()
            z_col = '#e5f5e0' if 'FREE' in z else ('#fff7bc' if 'MODERATE' in z else ('#fee391' if 'DENSE' in z else '#fcae91'))
            ax_speed.axvspan(t[i], t[i+1], color=z_col, alpha=0.35)

        plt.tight_layout()
        out_path = os.path.join(output_dir, f'{basename}_trajectory.png')
        plt.savefig(out_path, dpi=200)
        plt.close()

        fig_p, ax_p = plt.subplots(figsize=(10, 5))
        ax_p.plot(t, data['v_cmd'], label='v_cmd', color='#1f77b4', lw=2)
        ax_p.plot(t, v, label='v_act', color='#2ca02c', lw=2, ls='--')
        for i in range(len(t) - 1):
            z = data['zone'][i].upper()
            z_col = '#e5f5e0' if 'FREE' in z else ('#fff7bc' if 'MODERATE' in z else ('#fee391' if 'DENSE' in z else '#fcae91'))
            ax_p.axvspan(t[i], t[i+1], color=z_col, alpha=0.35)
        ax_p.set_title(f'Scenario 5 Speed Profile & Zone Shading ({basename})', fontsize=12, fontweight='bold')
        ax_p.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
        ax_p.set_ylabel('Velocity (m/s)', fontsize=10, fontweight='bold')
        ax_p.grid(True, linestyle=':', alpha=0.5)
        ax_p.legend(loc='upper right', fontsize=8)
        plt.tight_layout()
        prof_path = os.path.join(output_dir, f'{basename}_speed_profile.png')
        plt.savefig(prof_path, dpi=200)
        plt.close()
        print(f"[SUCCESS] Saved Scenario 5 trajectory and profile plots: {out_path}")

    elif is_scenario_6:
        fig, (ax_map, ax_speed) = plt.subplots(1, 2, figsize=(15, 6), gridspec_kw={'width_ratios': [1.3, 1]})
        draw_hospital_map(ax_map)

        scatter = ax_map.scatter(data['x'], data['y'], c=data['v_actual'], cmap='viridis', s=22, zorder=6, vmin=0.0, vmax=0.45)
        cbar = plt.colorbar(scatter, ax=ax_map, orientation='horizontal', pad=0.08, shrink=0.7)
        cbar.set_label('Actual Speed (m/s)', fontsize=10, fontweight='bold')

        ax_map.plot(data['x'][0], data['y'][0], marker='o', markersize=10, color='green', label='START', zorder=7)
        ax_map.plot(data['x'][-1], data['y'][-1], marker='*', markersize=14, color='gold', label='CURRENT / GOAL', zorder=7)
        ax_map.set_xlabel('Hospital X (m)', fontsize=10, fontweight='bold')
        ax_map.set_ylabel('Hospital Y (m)', fontsize=10, fontweight='bold')
        ax_map.set_title(f'Scenario 6: Hospital Trajectory ({basename})', fontsize=11, fontweight='bold')
        ax_map.legend(loc='lower left', fontsize=7, ncol=2)
        ax_map.grid(True, linestyle=':', alpha=0.4)

        t = data['elapsed_time_s']
        v = data['v_actual']
        ax_speed.plot(t, data['v_cmd'], label='Commanded (v_cmd)', color='#1f77b4', lw=2)
        ax_speed.plot(t, v, label='Actual (v_act)', color='#2ca02c', lw=2, ls='--')
        ax_speed.set_xlabel('Elapsed Time (s)', fontsize=10, fontweight='bold')
        ax_speed.set_ylabel('Speed (m/s)', fontsize=10, fontweight='bold')
        ax_speed.set_title(f'Velocity Profile & People Avoidance', fontsize=11, fontweight='bold')
        ax_speed.grid(True, linestyle=':', alpha=0.5)
        ax_speed.legend(loc='upper right', fontsize=8)

        for i in range(len(t) - 1):
            z = data['zone'][i].upper()
            z_col = '#e5f5e0' if 'FREE' in z else ('#fff7bc' if 'MODERATE' in z else ('#fee391' if 'DENSE' in z else '#fcae91'))
            ax_speed.axvspan(t[i], t[i+1], color=z_col, alpha=0.35)

        plt.tight_layout()
        out_path = os.path.join(output_dir, f'{basename}_trajectory.png')
        plt.savefig(out_path, dpi=200)
        plt.close()

        fig_p, ax_p = plt.subplots(figsize=(10, 5))
        ax_p.plot(t, data['v_cmd'], label='v_cmd', color='#1f77b4', lw=2)
        ax_p.plot(t, v, label='v_act', color='#2ca02c', lw=2, ls='--')
        for i in range(len(t) - 1):
            z = data['zone'][i].upper()
            z_col = '#e5f5e0' if 'FREE' in z else ('#fff7bc' if 'MODERATE' in z else ('#fee391' if 'DENSE' in z else '#fcae91'))
            ax_p.axvspan(t[i], t[i+1], color=z_col, alpha=0.35)
        ax_p.set_title(f'Scenario 6 Speed Profile & Zone Shading ({basename})', fontsize=12, fontweight='bold')
        ax_p.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
        ax_p.set_ylabel('Velocity (m/s)', fontsize=10, fontweight='bold')
        ax_p.grid(True, linestyle=':', alpha=0.5)
        ax_p.legend(loc='upper right', fontsize=8)
        plt.tight_layout()
        prof_path = os.path.join(output_dir, f'{basename}_speed_profile.png')
        plt.savefig(prof_path, dpi=200)
        plt.close()
        print(f"[SUCCESS] Saved Scenario 6 trajectory and profile plots: {out_path}")

    else:
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.axhline(1.8, color='black', linewidth=3, label='Corridor Walls (y=±1.8m)')
        ax.axhline(-1.8, color='black', linewidth=3)
        ax.axhline(0.0, color='gray', linestyle=':', alpha=0.5, label='Centerline (y=0.0m)')

        ax.axvspan(-10.0, -5.0, color='lightgreen', alpha=0.25, label='Zone 1: Empty Corridor')
        ax.axvspan(-5.0, -1.0, color='lightblue', alpha=0.25, label='Zone 2: Moderate Traffic')
        ax.axvspan(-1.0, 4.0, color='peachpuff', alpha=0.35, label='Zone 3: Cluttered Passage')
        ax.axvspan(4.0, 8.0, color='lightcoral', alpha=0.25, label='Zone 4: Critical Zone / Barrier')
        ax.axvspan(8.0, 10.5, color='gold', alpha=0.30, label='Goal Area')

        scatter = ax.scatter(data['x'], data['y'], c=data['v_actual'], cmap='viridis', s=16, zorder=5, vmin=0.0, vmax=0.45)
        cbar = plt.colorbar(scatter, ax=ax, orientation='vertical')
        cbar.set_label('Actual Speed (m/s)', fontsize=10, fontweight='bold')

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

        fig_p, ax_p = plt.subplots(figsize=(10, 4))
        t = data['elapsed_time_s']
        ax_p.plot(t, data['v_cmd'], label='v_cmd', color='#1f77b4', lw=2)
        ax_p.plot(t, data['v_actual'], label='v_act', color='#2ca02c', lw=2, ls='--')
        for i in range(len(t) - 1):
            z = data['zone'][i].upper()
            z_col = '#e5f5e0' if 'FREE' in z else ('#fff7bc' if 'MODERATE' in z else ('#fee391' if 'DENSE' in z else '#fcae91'))
            ax_p.axvspan(t[i], t[i+1], color=z_col, alpha=0.35)
        ax_p.set_title(f'Speed Profile & Zone Shading ({basename})', fontsize=12, fontweight='bold')
        ax_p.set_xlabel('Time (s)', fontsize=10, fontweight='bold')
        ax_p.set_ylabel('Velocity (m/s)', fontsize=10, fontweight='bold')
        ax_p.grid(True, linestyle=':', alpha=0.5)
        ax_p.legend(loc='upper right', fontsize=8)
        plt.tight_layout()
        prof_path = os.path.join(output_dir, f'{basename}_speed_profile.png')
        plt.savefig(prof_path, dpi=200)
        plt.close()
        print(f"[SUCCESS] Saved corridor trajectory & profile plots: {out_path}")


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
    print(f"{'Run / Preset':<28} {'Duration':<10} {'Mean Spd':<10} {'Max Spd':<10} {'Min Dist':<10} {'Stops'}")
    print("-" * 70)
    for r in summary_rows:
        print(f"{r[0]:<28} {r[1]:<10} {r[2]:<10} {r[3]:<10} {r[4]:<10} {r[5]}")
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


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else 'results'
    output_dir = sys.argv[2] if len(sys.argv) > 2 else 'docs/figures'

    os.makedirs(output_dir, exist_ok=True)
    csv_files = glob.glob(os.path.join(results_dir, "*.csv"))

    if not csv_files:
        print(f"[WARN] No CSV result files found in {results_dir}.")
        return

    print(f">> Processing {len(csv_files)} benchmark CSV files...")
    for f in csv_files:
        plot_speed_and_clutter(f, output_dir)
        plot_trajectory(f, output_dir)

    generate_comparison_summary(results_dir, output_dir)


if __name__ == '__main__':
    main()
