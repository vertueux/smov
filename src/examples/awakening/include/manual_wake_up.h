#pragma once

#include <time.h>   
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>

#include <smov/sequencer.h>

namespace smov {

class ManualWakeUpState {
 public:
  STATE_CLASS("Awakening")

  SequencerState seq = SequencerState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher);
};

} // namespace smov
