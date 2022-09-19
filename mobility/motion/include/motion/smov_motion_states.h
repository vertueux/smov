#pragma once

#include "i2cpwm_board_msgs/msg/servo_array.hpp"

namespace smov {

struct StateSwitch {
  bool walking;
  bool idle;
};

class StateAlgorithms {
 public:
  void walk(i2cpwm_board_msgs::msg::ServoArray servo_array);
};

}