#include <time.h>   
#include <unistd.h>
#include <iostream>

#include "manual_wake_up.h"

namespace smov {

void ManualWakeUpState::on_start() {
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_servos.value[i] = 0;
    back_servos.value[i] = 0;
    front_servos.value[i + (SERVO_MAX_SIZE / 3)] = 0.8;
    back_servos.value[i + (SERVO_MAX_SIZE / 3)] = 0.5;
    front_servos.value[i + 2 * (SERVO_MAX_SIZE / 3)] = 0.6f;
    back_servos.value[i + 2 * (SERVO_MAX_SIZE / 3)] = 0.6f;
  }

  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  std::vector<float> b_vals;
  for (float i = 0.8; i > 0.02; i -= 0.02) {
    std::cout << i << std::endl;
    b_vals.push_back(i);
  }

  seq.execute_muscles_sequence(BICEPS, b_vals, 50); // 150ms.

  std::vector<float> l_vals;
  for (float i = 0.56; i < 0.12f; i -= 0.02) {
    std::cout << i << std::endl;
    l_vals.push_back(i);
  }

  seq.execute_muscles_sequence(LEGS, l_vals, 50); // 150ms.

  // We end the program at the end.
  end_program();
}

void ManualWakeUpState::on_loop() {}

void ManualWakeUpState::on_quit() {}

}

DECLARE_STATE_NODE_CLASS("smov_awakening", smov::ManualWakeUpState, 500ms)
