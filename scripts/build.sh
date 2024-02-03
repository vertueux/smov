#!/bin/bash

if [[ -z "${ROS_DISTRO}" ]]; then
  echo "ERROR: ROS2 not sourced, please source ros2 then run again"
  exit 1
fi

echo "INFO: ROS_DISTRO found and set to ${ROS_DISTRO}"

if [ $1 = "dev" ]; then
  # Build to generate compile_commands.txt for ide usage
  echo "INFO: Building smov project with development compile_commands.txt"
  colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
else
  # Build without any special flags
  echo "INFO: Building smov project"
  colcon build
fi

echo "INFO: Complete"
