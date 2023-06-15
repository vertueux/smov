#include <states/smov_behaviors.h>

namespace smov {

FrontServos Behaviors::lock_front_phase_one() {
  FrontServos f_servos;
  f_servos = States::configure_front_servos();
  f_servos.front_servo0.value  = lockAbsoluteConfiguration.front_absolute_servo0_value;
  f_servos.front_servo15.value = lockAbsoluteConfiguration.front_absolute_servo15_value;
  return f_servos;
}

FrontServos Behaviors::lock_front_phase_two() {
  FrontServos f_servos;
  f_servos = States::configure_front_servos();
  f_servos.front_servo0.value  = lockAbsoluteConfiguration.front_absolute_servo0_value;
  f_servos.front_servo1.value  = lockAbsoluteConfiguration.front_absolute_servo1_value;
  f_servos.front_servo14.value = lockAbsoluteConfiguration.front_absolute_servo14_value;
  f_servos.front_servo15.value = lockAbsoluteConfiguration.front_absolute_servo15_value;
  return f_servos;
}

FrontServos Behaviors::lock_front_phase_three() {
  FrontServos f_servos;
  f_servos = States::configure_front_servos();
  f_servos.front_servo0.value  = lockAbsoluteConfiguration.front_absolute_servo0_value;
  f_servos.front_servo1.value  = lockAbsoluteConfiguration.front_absolute_servo1_value;
  f_servos.front_servo2.value  = lockAbsoluteConfiguration.front_absolute_servo2_value;
  f_servos.front_servo13.value = lockAbsoluteConfiguration.front_absolute_servo13_value;
  f_servos.front_servo14.value = lockAbsoluteConfiguration.front_absolute_servo14_value;
  f_servos.front_servo15.value = lockAbsoluteConfiguration.front_absolute_servo15_value;
  return f_servos;
}

BackServos Behaviors::lock_back_phase_one() {
  BackServos b_servos;
  b_servos = States::configure_back_servos();
  b_servos.back_servo0.value  = lockAbsoluteConfiguration.back_absolute_servo0_value;
  b_servos.back_servo15.value = lockAbsoluteConfiguration.back_absolute_servo15_value;
  return b_servos;
}

BackServos Behaviors::lock_back_phase_two() {
  BackServos b_servos;
  b_servos = States::configure_back_servos();
  b_servos.back_servo0.value  = lockAbsoluteConfiguration.back_absolute_servo0_value;
  b_servos.back_servo1.value  = lockAbsoluteConfiguration.back_absolute_servo1_value;
  b_servos.back_servo14.value = lockAbsoluteConfiguration.back_absolute_servo14_value;
  b_servos.back_servo15.value = lockAbsoluteConfiguration.back_absolute_servo15_value;
  return b_servos;
}
BackServos Behaviors::lock_back_phase_three() {
  BackServos b_servos;
  b_servos = States::configure_back_servos();
  b_servos.back_servo0.value  = lockAbsoluteConfiguration.back_absolute_servo0_value;
  b_servos.back_servo1.value  = lockAbsoluteConfiguration.back_absolute_servo1_value;
  b_servos.back_servo2.value  = lockAbsoluteConfiguration.back_absolute_servo2_value;
  b_servos.back_servo13.value = lockAbsoluteConfiguration.back_absolute_servo13_value;
  b_servos.back_servo14.value = lockAbsoluteConfiguration.back_absolute_servo14_value;
  b_servos.back_servo15.value = lockAbsoluteConfiguration.back_absolute_servo15_value;
  return b_servos;
}
void Behaviors::wake_up() {
  // Wake up.
  return;
}

}
