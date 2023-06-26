#include <rclcpp/rclcpp.hpp>
#include <states/smov_states.h>
#include <states/smov_behaviors.h>
 
#include <iostream>
namespace smov {

// Initializing default static values.
States *States::instance = nullptr;
FrontServoArray States::front_servos;
BackServoArray States::back_servos;
std::vector<std::vector<double>> States::front_servos_data;
std::vector<std::vector<double>> States::back_servos_data;

States::States() {}
States::~States() {}

States *States::Instance() {
  if (!instance) 
    instance = new States;
  return instance;
}

void States::set_up_servos(FrontServoArray f_servos, BackServoArray b_servos) {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    b_servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.
  }
}

void States::push_all_servos_in_array(FrontServoArray f_servos, BackServoArray b_servos) {
  States* node = States::Instance();

  // Clearing the vectors first.
  if (node->front_array.servos.size() > 0) 
    node->front_array.servos.clear();
  if (node->back_array.servos.size() > 0) 
    node->back_array.servos.clear();

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    node->front_array.servos.push_back(f_servos[i]);
    node->back_array.servos.push_back(b_servos[i]);
  }
}

} // namespace smov
