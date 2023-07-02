#include <states/smov_behaviors.h>

namespace smov {

void Behaviors::set_servos_to_center(FrontServoArray f_servos, BackServoArray b_servos) {
  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].value = States::front_servos_data[i][1]; // Center value is at position 1.
    b_servos[i].value = States::back_servos_data[i][1];
  }
}

void Behaviors::set_servos_to_min(FrontServoArray f_servos, BackServoArray b_servos) {
  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].value = States::front_servos_data[i][5]; // Minimum value is at position 6.
    b_servos[i].value = States::back_servos_data[i][5];
  }
}

void Behaviors::set_servos_to_max(FrontServoArray f_servos, BackServoArray b_servos) {
  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].value = States::front_servos_data[i][6]; // Minimum value is at position 7.
    b_servos[i].value = States::back_servos_data[i][6];
  }
}

}
