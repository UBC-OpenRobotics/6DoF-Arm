#!/usr/bin/env bash
set -e

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

exec "$@"
