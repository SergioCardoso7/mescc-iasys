#!/usr/bin/env bash

# IASYS environment checker
# Important: ROS 2 setup.bash is not guaranteed to be safe when Bash
# 'nounset' (set -u) is active. Therefore Jazzy is sourced before
# enabling nounset for the rest of this script.

ok=1

check_cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    printf '[OK]   %-12s %s\n' "$1" "$(command -v "$1")"
  else
    printf '[FAIL] %-12s not found\n' "$1"
    ok=0
  fi
}

echo 'IASYS environment check'
echo '-----------------------'

# ROS 2 commands are added to PATH by the ROS setup script. Source Jazzy
# here so the checker also works in a fresh terminal.
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # ROS setup scripts may reference variables that are not yet defined,
  # so make sure nounset is disabled while sourcing them.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  ros_source_status=$?

  if [ "$ros_source_status" -eq 0 ]; then
    echo '[OK]   ROS 2 Jazzy installation found and sourced from /opt/ros/jazzy'
  else
    echo '[FAIL] ROS 2 Jazzy setup script returned an error.'
    ok=0
  fi
else
  echo '[FAIL] ROS 2 Jazzy not found in /opt/ros/jazzy'
  ok=0
fi

# Enable stricter handling only after the ROS environment has been sourced.
set -u

echo
check_cmd docker
check_cmd ade
check_cmd ros2
check_cmd colcon
check_cmd rviz2

echo
if [ "${ROS_DISTRO:-}" = "jazzy" ]; then
  echo '[OK]   Active ROS distribution: jazzy'
elif [ -n "${ROS_DISTRO:-}" ]; then
  echo "[WARN] Active ROS distribution is '${ROS_DISTRO}', expected 'jazzy'."
else
  echo '[FAIL] ROS_DISTRO is not set after sourcing Jazzy.'
  ok=0
fi

if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
  echo '[OK]   Docker daemon accessible without sudo'
elif command -v docker >/dev/null 2>&1; then
  echo '[WARN] Docker is installed but the daemon is unavailable or your user lacks permission.'
else
  echo '[WARN] Docker daemon check skipped because Docker is not installed.'
fi

if [ -d "$HOME/adehome/AutowareAuto" ]; then
  echo '[OK]   Autoware.Auto checkout found at ~/adehome/AutowareAuto'
else
  echo '[WARN] Autoware.Auto checkout not found at ~/adehome/AutowareAuto'
fi

echo
if [ "$ok" -eq 1 ]; then
  echo 'Core IASYS environment looks ready.'
  exit 0
else
  echo 'One or more required components are missing.'
  echo 'If only rviz2 is missing after Jazzy was sourced, install it with:'
  echo '  sudo apt install ros-jazzy-rviz2'
  exit 1
fi
