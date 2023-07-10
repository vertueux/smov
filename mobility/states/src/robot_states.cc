#include <states/robot_states.h>

namespace smov {

RobotManager **States::get_robot() {
  RobotManager* robot = RobotManager::Instance();
  return &robot;
}

}
