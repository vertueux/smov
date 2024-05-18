#ifndef SET_LEGS_DISTANCE_TO_H_
#define SET_LEGS_DISTANCE_TO_H_

#include <errno.h>   // for errno.
#include <limits.h>  // for INT_MAX, INT_MIN.
#include <stdlib.h>  // for strtol.

#include <rclcpp/rclcpp.hpp>

#include <smov/executable.h>

#include <smov/trigonometry.h>

namespace smov {

int _argc;
char **_argv;

class LegsDistanceState {
 public:
  STATE_CLASS(LegsDistanceState)
  double leg_width = 2.5;
          
  TrigonometryState trig = TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &leg_width, &hip_body_distance);
  int desired_distance = 10;
};

} // namespace smov

#endif // SET_LEGS_DISTANCE_TO_H_
