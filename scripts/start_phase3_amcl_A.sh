#!/usr/bin/env bash
# Condition A: stock model TF and AMCL. Refuse if any existing sim running.
set -eo pipefail
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
export TURTLEBOT3_MODEL=waffle
if pgrep -x gzserver >/dev/null || pgrep -x ekf_node >/dev/null; then
  echo "Refusing: another Gazebo/EKF process is running. Do not run trials concurrently." >&2
  exit 2
fi
if timeout 5 ros2 node list 2>/dev/null | grep -Eq '^/(amcl|bt_navigator|controller_server|ekf_shadow|ekf_assisted)$'; then
  echo "Refusing: Nav2 or EKF nodes already active." >&2
  exit 2
fi
echo "Starting controlled A: Gazebo odom TF + AMCL + matched 0.10m goals"
exec ros2 launch "$BASE/launch/task2_amcl_baseline.launch.py" "$@"
