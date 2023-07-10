#include <states/robot_states.h>

namespace smov {

RobotManager **States::get_robot() {
  RobotManager* node = RobotManager::Instance();
  return &node;
}

}
