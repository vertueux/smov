#pragma once

#include <time.h>   
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

#include <smov/trigonometry.h>

namespace smov {

class BreathState {
 public:
  STATE_CLASS("Breath")
  
  void sleep_in_milliseconds(int time);

  std::array<std::array<float, 2>, 12> data = {{{0, 120},{0, 120},{55, 145},{55, 145},{70, 150},{70, 150},  // Front servos.
                                               {0, 120},{0, 120},{55, 145},{55, 145},{70, 150},{70, 150}}}; // Back servos.               
  TrigonometryState trig = TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, 14, 14, 2.5f, data);
  struct timespec ts;
};

} // namespace smov
