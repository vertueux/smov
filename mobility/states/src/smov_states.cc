#include <rclcpp/rclcpp.hpp>
#include <states/smov_states.h>
#include <states/smov_behaviors.h>
 
#include <iostream>
namespace smov {

// Initializing default static values.
States *States::instance = nullptr;
FrontServoArray States::front_prop_servos;
BackServoArray States::back_prop_servos;
FrontServoArray States::front_abs_servos;
BackServoArray States::back_abs_servos;
std::vector<std::vector<long int>> States::front_servos_data;
std::vector<std::vector<long int>> States::back_servos_data;
double States::pulse_for_angle = 0.0;

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

void States::push_in_abs_array(FrontServoArray f_servos, BackServoArray b_servos) {
  States* node = States::Instance();

  // Clearing the vectors first.
  if (node->front_abs_array.servos.size() > 0) 
    node->front_abs_array.servos.clear();
  if (node->back_abs_array.servos.size() > 0) 
    node->back_abs_array.servos.clear();

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    node->front_abs_array.servos.push_back(f_servos[i]);
    node->back_abs_array.servos.push_back(b_servos[i]);
  }
}

void States::push_in_prop_array(FrontServoArray f_servos, BackServoArray b_servos) {
  States* node = States::Instance();

  // Clearing the vectors first.
  if (node->front_prop_array.servos.size() > 0) 
    node->front_prop_array.servos.clear();
  if (node->back_prop_array.servos.size() > 0) 
    node->back_prop_array.servos.clear();

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    node->front_prop_array.servos.push_back(f_servos[i]);
    node->back_prop_array.servos.push_back(b_servos[i]);
  }
}

} // namespace smov
