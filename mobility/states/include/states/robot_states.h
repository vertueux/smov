#pragma once

#include <string>

#include <states/robot_manager.h>

namespace smov {

#define STATE_NAME(name) virtual const char* get_name() const override { return #name; }

class State {
 public:
  virtual RobotManager **get_robot() {return &robot;};
  virtual const char* get_name() const = 0;

 private:
  RobotManager* robot = RobotManager::Instance();
};

} // namespace smov
