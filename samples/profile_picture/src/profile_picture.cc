#include <profile_picture/profile_picture.h>
#include <iostream>

namespace smov {

void ProfilePicture::on_start() {
  // Default value when awakened.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_servos.value[0] = 0.3f;
    front_servos.value[1] = -0.3f;
    front_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f; // -2/3
    front_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;

    back_servos.value[0] = 0.3f;
    back_servos.value[1] = -0.3f;
    back_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f;
    back_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }

  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);
}

void ProfilePicture::on_loop() {
}

void ProfilePicture::on_quit() {
}

}

DECLARE_STATE_NODE_CLASS("smov_manual_walk_state", smov::ProfilePicture, 10ms)
