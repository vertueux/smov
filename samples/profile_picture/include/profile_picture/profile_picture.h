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
};

} // namespace smov
