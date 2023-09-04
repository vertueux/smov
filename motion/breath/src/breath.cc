#include <breath/breath.h>

namespace smov {

void BreathState::on_start() {
}

void BreathState::on_loop() {
  trig.set_legs_distance_to(8); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(9); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(10); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(11); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(12); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(13); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(14); // 14 cm.

  // Sleeping for a short period of time.
  delay(800);

  trig.set_legs_distance_to(13); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(12); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(11); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(10); 

  // Sleeping for a short period of time.
  delay(100);

  trig.set_legs_distance_to(9); 
}

void BreathState::on_quit() {
}

}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("smov_breath_state", smov::BreathState, 500ms)
