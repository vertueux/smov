#include "smov_behaviors.h"

namespace smov {

void Behaviors::lock_all_front_servos(FrontServos f_servos) {
  f_servos.front_servo0.value  = lockAbsoluteConfiguration.front_absolute_servo0_value;
  f_servos.front_servo1.value  = lockAbsoluteConfiguration.front_absolute_servo1_value;
  f_servos.front_servo2.value  = lockAbsoluteConfiguration.front_absolute_servo2_value;
  f_servos.front_servo13.value = lockAbsoluteConfiguration.front_absolute_servo13_value;
  f_servos.front_servo14.value = lockAbsoluteConfiguration.front_absolute_servo14_value;
  f_servos.front_servo15.value = lockAbsoluteConfiguration.front_absolute_servo15_value;
}

void Behaviors::lock_all_back_servos(BackServos b_servos) {
  b_servos.back_servo0.value  = lockAbsoluteConfiguration.back_absolute_servo0_value;
  b_servos.back_servo1.value  = lockAbsoluteConfiguration.back_absolute_servo1_value;
  b_servos.back_servo2.value  = lockAbsoluteConfiguration.back_absolute_servo2_value;
  b_servos.back_servo13.value = lockAbsoluteConfiguration.back_absolute_servo13_value;
  b_servos.back_servo14.value = lockAbsoluteConfiguration.back_absolute_servo14_value;
  b_servos.back_servo15.value = lockAbsoluteConfiguration.back_absolute_servo15_value;
}

}
