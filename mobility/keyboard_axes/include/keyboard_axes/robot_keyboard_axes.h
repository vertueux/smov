#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

#define KEY_UP 65
#define KEY_DOWN 66
#define KEY_RIGHT 67
#define KEY_LEFT 68


namespace smov {

class KeyboardAxesState {
 public:
  STATE_CLASS("Keyboard Axes")

  // Useful to read characters without blocking the program.
  void init_reader(int echo);

  float biceps_value = 0.0;
  float body_value = 0.0;
  float leg_value = 1.0;

  struct termios old_chars, new_chars;
};

} // namespace smov
