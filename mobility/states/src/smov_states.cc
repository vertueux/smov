#include <rclcpp/rclcpp.hpp>
#include <states/smov_states.h>
 
namespace smov {

States *States::instance = nullptr;

States::States() {}
States::~States() {}

States *States::Instance() {
  if (!instance) 
    instance = new States;
  return instance;
}

FrontServos States::configure_front_servos() {
  FrontServos f_servos;

  // Front left configuration.
  f_servos.front_servo0.servo = 1;
  f_servos.front_servo0.value = 0;

  f_servos.front_servo1.servo = 2;
  f_servos.front_servo1.value = 0;

  f_servos.front_servo2.servo = 3;
  f_servos.front_servo2.value = 0;

  // Front right configuration.
  f_servos.front_servo13.servo = 14;
  f_servos.front_servo13.value = 0;

  f_servos.front_servo14.servo = 15;
  f_servos.front_servo14.value = 0;

  f_servos.front_servo15.servo = 16;
  f_servos.front_servo15.value = 0;

  return f_servos;
}

BackServos States::configure_back_servos() {
  BackServos b_servos;

  // Back left configuration.
  b_servos.back_servo0.servo = 1;
  b_servos.back_servo0.value = 0;

  b_servos.back_servo1.servo = 2;
  b_servos.back_servo1.value = 0;

  b_servos.back_servo2.servo = 3;
  b_servos.back_servo2.value = 0;

  // Back right configuration.
  b_servos.back_servo13.servo = 14;
  b_servos.back_servo13.value = 0;

  b_servos.back_servo14.servo = 15;
  b_servos.back_servo14.value = 0;

  b_servos.back_servo15.servo = 16;
  b_servos.back_servo15.value = 0;
  
  return b_servos;
}


void States::push_all_front_servos_in_array(FrontServos f_servos) {
  States* node = States::Instance();
  node->front_array.servos.push_back(f_servos.front_servo0);
  node->front_array.servos.push_back(f_servos.front_servo1);
  node->front_array.servos.push_back(f_servos.front_servo2);
  node->front_array.servos.push_back(f_servos.front_servo13);
  node->front_array.servos.push_back(f_servos.front_servo14);
  node->front_array.servos.push_back(f_servos.front_servo15);
}

void States::push_all_back_servos_in_array(BackServos b_servos) {
  States* node = States::Instance();
  node->back_array.servos.push_back(b_servos.back_servo0);
  node->back_array.servos.push_back(b_servos.back_servo1);
  node->back_array.servos.push_back(b_servos.back_servo2);
  node->back_array.servos.push_back(b_servos.back_servo13);
  node->back_array.servos.push_back(b_servos.back_servo14);
  node->back_array.servos.push_back(b_servos.back_servo15);
}

} // namespace smov
