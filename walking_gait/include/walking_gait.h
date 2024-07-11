#pragma once

#include <math.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include "smov/executable.h"
#include "smov/trigonometry.h"

enum Mode {
  STANDING = 0,
  WALKING = 1,
  TURNING = 2
};

class ForwardMotion {
 public:
  STATE_CLASS(ForwardMotion)

  // Mathematical function. f(x) = -sqrt(25 - ((2x) - 2)^2) + 23 + gap.
  float curved(float x, float gap);
  void stabilize_legs();
  void output_coordinates();

  // Used for modes.
  void walk();
  void turn_right();

  float back_leg_gap = 3.0f;
  Mode mode = STANDING;
  smov::TrigonometryState trig = smov::TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &hip_body_distance);
  smov::Vector3 coord1, coord2, coord3, coord4;
  bool leg1_motion_done = true, leg2_motion_done = true, leg3_motion_done = true, leg4_motion_done = true;
  bool has_finished_walk = false, done_once = false, request_to_stop_walk = false;
  float i1 = 0, i2 = 0, i3 = 0, i4 = 0;

  // Used for reading terminal values.
  struct termios old_chars, new_chars;
};
