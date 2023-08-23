#include <breath/breath.h>

namespace smov {

void BreathState::on_start() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_servos.value[i] = 0.0f;
    back_servos.value[i] = 0.0f;
  }

  trig = TrigonometryState(&front_servos, &back_servos,
   &front_state_publisher, &back_state_publisher, data, 14, 14); // Mine are 14cm both.
  trig.set_legs_distance_to(8); // 8 cm.
}

void BreathState::on_loop() {
}

void BreathState::on_quit() {
}

}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("smov_breath_state", smov::BreathState, 500ms)
