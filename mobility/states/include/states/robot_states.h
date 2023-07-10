#pragma once

#include <string>

#include <states/robot_manager.h>

namespace smov {

class State {
 public:
  virtual RobotManager **get_robot() {return &node;};

 private:
  RobotManager* node = RobotManager::Instance();
};

} // namespace smov
