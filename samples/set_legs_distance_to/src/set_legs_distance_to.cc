#include <set_legs_distance_to/set_legs_distance_to.h>

namespace smov {
  
void LegsDistanceState::on_start() {
  char *p;
  errno = 0;
  if (_argv[1] != NULL) {
    long conv = strtol(_argv[1], &p, 10);
    if (errno != 0 || *p != '\0' || conv > INT_MAX || conv < INT_MIN) 
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Expecting an integer to set the distance.");
    else 
      desired_distance = conv;
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "No distance specified, defaulting to 10cm.");
    desired_distance = 10;
  }

	trig.set_legs_distance_to(desired_distance); // In centimeters.

  // We end the program at the end.
  end_program();
}

void LegsDistanceState::on_loop() {
}

void LegsDistanceState::on_quit() {
}

}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS_GET_ARGS("smov_breath_state", smov::LegsDistanceState, 500ms, smov::_argc, smov::_argv)
