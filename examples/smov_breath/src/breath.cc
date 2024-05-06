#include "breath.h"

namespace smov {

void BreathState::on_start() {
}

void BreathState::on_loop() {
  for (int i = 8; i < 20; i++) {
    trig.set_legs_distance_to(i); 
    delay(50);
  }

  for (int i = 20; i > 8; i--) {
    trig.set_legs_distance_to(i); 
    delay(50);
  }
}

void BreathState::on_quit() {
}

}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("smov_breath_state", smov::BreathState, 500ms)
