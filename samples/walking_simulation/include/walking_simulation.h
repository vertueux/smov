#pragma once

#include <rclcpp/rclcpp.hpp>

#include <smov/executable.h>
#include <smov/trigonometry.h>

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_RIGHT 67
#define KEY_LEFT 68

class WalkingSimulationState {
 public:
  STATE_CLASS(WalkingSimulationState)
  
  void initialize_coordinates();

  smov::TrigonometryState trig = smov::TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &leg_width, &hip_body_distance);
  double leg_width = 2.5;
  smov::Vector3 coord1;
  smov::Vector3 coord2;
  smov::Vector3 coord3;
  smov::Vector3 coord4;
};