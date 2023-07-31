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

class ProfilePicture {
 public:
  STATE_CLASS("Profile Picture")

  void smooth_transition(float &receiver1, float &receiver2, float &receiver3, float &receiver4, float value);
  void animate_first_part();
  void animate_second_part();
  int cooldown = 2;
};

} // namespace smov
