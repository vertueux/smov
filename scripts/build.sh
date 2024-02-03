#!/bin/bash

if [[ -z "${ROS_DISTRO}" ]]; then
  echo "ERROR: ROS2 not sourced, please source ros2 then run again"
  exit 1
fi

echo "INFO: ROS_DISTRO found and set to ${ROS_DISTRO}"

# Simple script to be expanded later
echo "INFO: Building smov project"
colcon build
echo "INFO: Complete"
