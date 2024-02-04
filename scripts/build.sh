#!/bin/bash

if [[ -z "${ROS_DISTRO}" ]]; then
  echo "ERROR: ROS2 not sourced, please source ros2 then run again"
  exit 1
fi

echo "INFO: ROS_DISTRO found and set to ${ROS_DISTRO}"

if [ $# -eq 0 ]; then
  # Build without any special flags
  echo "INFO: Building smov project"
  colcon build
  exit
fi


if [ $1 = "devall" ]; then
  # Build to generate compile_commands.txt for ide usage
  echo "INFO: Building smov project with development compile_commands.txt"
  colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
else
  # Build without any special flags
  echo "INFO: Building smov package $1"
  colcon build --packages-select $1 --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON 
fi