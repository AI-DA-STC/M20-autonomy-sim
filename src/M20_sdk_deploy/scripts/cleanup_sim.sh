#!/usr/bin/env bash
set -euo pipefail

signal="-TERM"
if [[ "${1:-}" == "--force" || "${1:-}" == "-9" ]]; then
  signal="-KILL"
fi

patterns=(
  "ros2 topic pub"
  "parameter_bridge"
  "ros_gz"
  "rviz2"
  "pointlio_mapping"
  "far_planner"
  "localPlanner"
  "pathFollower"
)

for pattern in "${patterns[@]}"; do
  if pgrep -f "$pattern" >/dev/null; then
    echo "Killing ($signal): $pattern"
    pkill "$signal" -f "$pattern" || true
  else
    echo "Not running: $pattern"
  fi
done

echo "Cleanup complete."
