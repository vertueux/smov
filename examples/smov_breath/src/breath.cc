#include "breath.h"

#define MAX 20
#define MIN 8

namespace smov {

void BreathState::on_start() {
}

void BreathState::on_loop() {
  for (int i = MIN; i < MAX; i++) {
    trig.set_legs_distance_to(i); 
    delay(50);
  }

  for (int i = MAX; i > MIN; i--) {
    trig.set_legs_distance_to(i); 
    delay(50);
  }
}

void BreathState::on_quit() {
}

}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("smov_breath_state", smov::BreathState, 500ms)
