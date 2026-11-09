#!/usr/bin/env bash
# B configuration only; do not run concurrently with original Gazebo/Nav2.
set -eo pipefail
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
set -u
export TURTLEBOT3_MODEL=waffle
LIB="$BASE/deps/extracted/opt/ros/humble/lib"
if [[ -f /opt/ros/humble/lib/libdiagnostic_updater.so ]]; then
  echo "Using installed diagnostic_updater shared library"
elif [[ -f "$LIB/libdiagnostic_updater.so" ]]; then
  export LD_LIBRARY_PATH="$LIB:${LD_LIBRARY_PATH:-}"
  echo "Using project-local diagnostic_updater shared library"
else
  echo "Missing EKF runtime shared library; refusing launch" >&2
  exit 1
fi
if pgrep -x gzserver >/dev/null || pgrep -x ekf_node >/dev/null; then
  echo "Refusing: existing Gazebo or EKF process detected. Stop baseline cleanly before B experiment." >&2
  exit 2
fi
if timeout 5 ros2 node list 2>/dev/null | grep -Eq '^/(amcl|bt_navigator|controller_server|ekf_shadow|ekf_assisted)$'; then
  echo "Refusing: existing ROS navigation/EKF nodes detected." >&2
  exit 2
fi
echo "Starting B: stock world, copied SDF with Gazebo odom TF off, EKF TF on, filtered odometry"
exec ros2 launch "$BASE/launch/task2_ekf_assisted.launch.py" "$@"
