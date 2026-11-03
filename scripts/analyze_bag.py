#!/usr/bin/env python3
"""Offline rosbag2 extractor for controller telemetry topics.

Requires rosbag2_py in a sourced ROS 2 environment. It writes a CSV that can
be passed to analyze_results.py; unsupported message types are skipped.
"""
import argparse
import csv
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--output', default='results/bag_metrics.csv')
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = Path(args.output); out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['time', 'topic', 'speed', 'clearance', 'density_rho', 'zone', 'limit_reason'])
        writer.writeheader()
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic not in ('/robot_status', '/obstacle_info', '/cmd_vel'):
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            writer.writerow({'time': stamp / 1e9, 'topic': topic, 'speed': getattr(msg, 'speed', getattr(getattr(msg, 'linear', None), 'x', '')), 'clearance': getattr(msg, 'clearance', ''), 'density_rho': getattr(msg, 'density_rho', ''), 'zone': getattr(msg, 'zone', ''), 'limit_reason': getattr(msg, 'limit_reason', '')})
    print(out)


if __name__ == '__main__':
    main()
