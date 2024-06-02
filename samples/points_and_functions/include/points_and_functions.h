#ifndef SET_LEGS_DISTANCE_TO_H_
#define SET_LEGS_DISTANCE_TO_H_

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>

#include <smov/executable.h>
#include <smov/trigonometry.h>

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_RIGHT 67
#define KEY_LEFT 68

class PointsAndFunctionsState {
 public:
  STATE_CLASS(PointsAndFunctionsState)

  // Useful to read characters without blocking the program.
  void init_reader(int echo);

  smov::TrigonometryState trig = smov::TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &leg_width, &hip_body_distance);
  double leg_width = 2.5;
  smov::Vector3 coord;
  struct termios old_chars, new_chars;
};

#endif // SET_LEGS_DISTANCE_TO_H_
