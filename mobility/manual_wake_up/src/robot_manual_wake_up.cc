#include <unistd.h>

#include <manual_wake_up/robot_manual_wake_up.h>

namespace smov {

void ManualWakeUpState::on_start() {
  // We add a tiny cooldown for safety & to make sure
  // it does the command succesfully.
  sleep(cooldown);
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_values.value[i] = 0.0f; 
    back_values.value[i] = 0.0f; 
  }
  front_state_publisher->publish(front_values);
  back_state_publisher->publish(back_values);
  sleep(cooldown);
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_values.value[i + SERVO_MAX_SIZE / 3] = -0.066667f; // -2/3
    back_values.value[i + SERVO_MAX_SIZE / 3] = -0.066667f; 
    front_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.122221f; 
    back_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.122221f; 
  }
}

void ManualWakeUpState::on_loop() {}

void ManualWakeUpState::on_quit() {}

}