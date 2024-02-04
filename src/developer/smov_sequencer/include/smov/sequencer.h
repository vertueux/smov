#pragma once

#include <vector>
#include <array>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

enum ServoOrder {
  BODY_BICEPS_LEGS = 0,
  BODY_LEGS_BICEPS = 1,
  BICEPS_LEGS_BODY = 2,
  BICEPS_BODY_LEGS = 3,
  LEGS_BODY_BICEPS = 4,
  LEGS_BICEPS_BODY = 5,
};

enum RobotMuscles {
  BODY = 0,
  BICEPS = 1,
  LEGS = 2,
};

class SequencerState {
 public:
  STATE_LIBRARY_CLASS(SequencerState)

  void execute_muscles_sequence(RobotMuscles group, const std::vector<float>& values, int cool_down);
  void execute_sequence(MicroController mc, int servo, const std::vector<float>& values, int cool_down);
  void execute_sequence(MicroController mc, std::array<int, 2> servos, const std::vector<float>& values, int cool_down);
  void execute_group_sequence(MicroController mc, ServoOrder sequence, std::array<float, 3> values, int timeout);
  void execute_global_sequence(MicroController mc, std::vector<float> values, int cool_down);
};

} // namespace smov
