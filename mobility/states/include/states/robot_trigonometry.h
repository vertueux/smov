#pragma once

#include <states/robot_states.h>

namespace smov {

class TrigonometryState : public State {
 public:
  STATE_NAME("Trigonometry")

  void set_legs_distance(double distance);
};

} // namespace smov
