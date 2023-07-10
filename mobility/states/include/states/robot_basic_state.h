#pragma once

#include <states/robot_manager.h>
#include <states/robot_states.h>

namespace smov {

enum ServoOrder {
  BODY_BICEPS_LEGS = 0,
  BODY_LEGS_BICEPS = 1,
  BICEPS_LEGS_BODY = 2,
  BICEPS_BODY_LEGS = 3,
  LEGS_BODY_BICEPS = 4,
  LEGS_BICEPS_BODY = 5,
};

class BasicState : public State {
 public:
  //RobotManager* node = RobotManager::Instance();

  // Used for proportional servos.
  void set_servos_to(double value); 
  void set_servos_to_center(); 

  // Both are used for absolute servos.
  void set_servos_to_min(); 
  void set_servos_to_max(); 
};

} // namespace smov
