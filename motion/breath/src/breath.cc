#include <breath/breath.h>
#include <iostream>

namespace smov {

void BreathState::on_start() {
  trig.set_legs_distance_to(8); // 8 cm.
}

void BreathState::on_loop() {
}

void BreathState::on_quit() {
}

}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("smov_breath_state", smov::BreathState, 500ms)
