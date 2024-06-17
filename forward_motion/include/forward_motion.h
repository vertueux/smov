#pragma once

#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include "executable.h"
#include "trigonometry.h"

class ForwardMotion {
 public:
  STATE_CLASS(ForwardMotion)

  // Mathematical function. f(x) = -sqrt(25 - ((x / 2) - 5)^2) + 20.
  float curved(float x);
  float curved_gap(float x);

  void stabilize_legs();
  void output_coordinates();

  smov::TrigonometryState trig = smov::TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &hip_body_distance);
  smov::Vector3 coord1, coord2, coord3, coord4;
  bool leg1_motion_done = false, leg2_motion_done = true, leg3_motion_done = true, leg4_motion_done = false;
  float i1 = 0, i2 = 0, i3 = 0, i4 = 0;
};
