#pragma once

#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

enum MicroController {
  FRONT = 0,
  BACK = 1
};

enum ServoOrder {
  BODY_BICEPS_LEGS = 0,
  BODY_LEGS_BICEPS = 1,
  BICEPS_LEGS_BODY = 2,
  BICEPS_BODY_LEGS = 3,
  LEGS_BODY_BICEPS = 4,
  LEGS_BICEPS_BODY = 5,
};

class SequencerState {
 public:
  STATE_LIBRARY_CLASS(SequencerState)

  void smooth_transition(float& var, float result, int accuracy, int speed);
  void execute_sequence(MicroController mc, int servo, std::vector<float> values, int cooldown);
  void execute_sequence(MicroController mc, std::array<int, 2> servos, std::vector<float> values, int cooldown);
  void execute_group_sequence(MicroController mc, ServoOrder sequence, std::array<float, 3> values, int timeout);
  void execute_global_sequence(MicroController mc, std::vector<float> values, int cooldown);
};

} // namespace smov
