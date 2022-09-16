#include "simulation/spot_micro_control.h"

using namespace std;

namespace smov {

SimulationControl::SimulationControl()
  : Node("command_control") {
  while (rclcpp::ok()) {
    switch(str2int(input)) {
      case str2int("right"):
        temp_legs.legs = {0,0,1,1};
        move_to(temp_legs, positions.sit_back);
        temp_legs.legs = {1,1,0,0};
        move_to(temp_legs,positions.sit_front);
        break;
      case str2int("up"):
        temp_legs.legs = {1,1,1,1};
        move_to(temp_legs, positions.up);
        break;
      case str2int("down"):
        temp_legs.legs = {1,1,1,1};
        move_to(temp_legs, positions.crouched);
        break;
    }
  }
}

void SimulationControl::move_to(Leg leg, vector<double> position) {
  vector<double> angles = kinematics.leg_kinematics(position);

  leg.angles[0] = angles[0];
  leg.angles[1] = angles[1];
  leg.angles[2] = angles[2];
}

}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::SimulationControl>());
  rclcpp::shutdown();
  return 0;
}