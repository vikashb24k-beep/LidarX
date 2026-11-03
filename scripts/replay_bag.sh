#!/usr/bin/env bash
# Replay a recorded experiment against nodes configured with use_sim_time:=true.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <bag-directory>" >&2
  exit 2
fi
ros2 bag play --clock "$1"
