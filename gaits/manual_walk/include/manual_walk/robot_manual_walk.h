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

class ManualWalk {
 public:
  STATE_CLASS("Manual Walk")

  void init_reader(int echo);

  void execute_forward_sequence();
  void execute_backward_sequence();
  void execute_right_sequence();
  void execute_left_sequence();

  bool request_front_walk = false;
  bool request_back_walk  = false;
  bool request_right_walk = false;
  bool request_left_walk  = false;

  int cooldown = 1;
  struct termios old_chars, new_chars;
};

} // namespace smov
