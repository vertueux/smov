#include <states/smov_behaviors.h>

namespace smov {

void Behaviors::lock_servos(FrontServoArray f_servos, BackServoArray b_servos) {
  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].servo = States::front_servos_data[i][1]; // Center value is at position 1.
    b_servos[i].servo = States::back_servos_data[i][1];
  }
}

}
