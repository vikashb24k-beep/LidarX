#!/usr/bin/env python3
"""Summarise per-run experiment CSV files without hiding failed runs."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev


def number(row, *names):
    for name in names:
        try:
            return float(row[name])
        except (KeyError, TypeError, ValueError):
            pass
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('results_dir', nargs='?', default='results')
    parser.add_argument('--output', default='docs/results/summary.md')
    args = parser.parse_args()
    groups = defaultdict(list)
    for path in Path(args.results_dir).glob('*.csv'):
        with path.open(newline='') as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            continue
        first, last = rows[0], rows[-1]
        scenario = first.get('scenario', path.stem.split('_')[0])
        policy = first.get('speed_policy', first.get('policy', 'unknown'))
        values = {
            'time_s': number(last, 'elapsed_time', 'time'),
            'min_clearance_m': min((v for v in (number(r, 'clearance') for r in rows) if v is not None), default=None),
            'collisions': number(last, 'collisions', 'collision_count'),
            'near_misses': number(last, 'near_misses', 'near_miss_count'),
            'avg_speed_mps': mean(v for v in (number(r, 'speed', 'actual_speed') for r in rows) if v is not None),
        }
        groups[(scenario, policy)].append(values)
    lines = ['# Experiment summary', '', '| Scenario | Policy | n | Time (s) | Avg speed (m/s) | Min clearance (m) | Collisions | Near misses |', '|---|---|---:|---:|---:|---:|---:|---:|']
    for (scenario, policy), runs in sorted(groups.items()):
        def fmt(key):
            vals = [r[key] for r in runs if r[key] is not None]
            if not vals:
                return 'n/a'
            return f'{mean(vals):.3f}' + (f' ± {stdev(vals):.3f}' if len(vals) > 1 else '')
        lines.append(f'| {scenario} | {policy} | {len(runs)} | {fmt("time_s")} | {fmt("avg_speed_mps")} | {fmt("min_clearance_m")} | {fmt("collisions")} | {fmt("near_misses")} |')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines) + '\n')
    print(output)


if __name__ == '__main__':
    main()
