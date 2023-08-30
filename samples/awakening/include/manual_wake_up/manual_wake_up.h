#pragma once

#include <time.h>   
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

class ManualWakeUpState {
 public:
  STATE_CLASS("Awakening")

  void sleep_in_milliseconds(int time);
  struct timespec ts;
  int cooldown = 700; // 700 milliseconds.
};

} // namespace smov
