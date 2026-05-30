#!/usr/bin/env bash
set -eo pipefail

cd /workspaces/bcr_arm

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"

restore_nounset=0
case $- in
  *u*)
    restore_nounset=1
    set +u
    ;;
esac

source /opt/ros/humble/setup.bash

if [ -f /usr/share/gazebo-11/setup.sh ]; then
  source /usr/share/gazebo-11/setup.sh
fi

if [ "${restore_nounset}" -eq 1 ]; then
  set -u
fi

if ! grep -Rqs "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
    > /etc/apt/sources.list.d/ros2.list
fi

apt-get update
rosdep update
mapfile -t package_paths < <(colcon list --packages-up-to bcr_arm_rx150 --paths-only)
rosdep install --from-paths "${package_paths[@]}" --ignore-src -r -y
colcon build --packages-up-to bcr_arm_rx150 --symlink-install
