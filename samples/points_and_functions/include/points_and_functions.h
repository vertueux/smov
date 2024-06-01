#ifndef SET_LEGS_DISTANCE_TO_H_
#define SET_LEGS_DISTANCE_TO_H_

#include <rclcpp/rclcpp.hpp>

#include <smov/executable.h>
#include <smov/trigonometry.h>

int _argc;
char **_argv;

class PointsAndFunctionsState {
 public:
  STATE_CLASS(PointsAndFunctionsState)
  double leg_width = 2.5;
};

#endif // SET_LEGS_DISTANCE_TO_H_
