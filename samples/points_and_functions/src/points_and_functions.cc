#include <math.h>

#include "points_and_functions.h"

void PointsAndFunctionsState::on_start() {
  for (int i = 0; i < 6; i++) {
    front_servos.value[i] = 0.0f;
  }

  front_state_publisher->publish(front_servos);

  front_servos.value[1] = 90.0f;
  front_servos.value[3] = 55.0f;
  front_servos.value[5] = 30.0f; // (val - base) / (max - base) = -1

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting default position to (15, 15, 10)");
  coord.x = 14;
  coord.y = 15;
  coord.z = 5;
  trig.set_leg_to(1, coord);
}

void PointsAndFunctionsState::on_loop() {
  // Creating a function: f(x) = -sqrt(25 - ((coord.x / 2) - 5)^2) + 20.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
  coord.y = -sqrt(25.0f - pow(((coord.x / 2) - 2.0f), 2)) + 15.0f; // We have to multiply by -1 to turn upside down the curve to match  
  if (coord.x >= -6)
    coord.x -= 1.0f;
  else {
    coord.x = 14.0f;
    coord.y = 15.0f;
  }
  trig.set_leg_to(1, coord);
}

void PointsAndFunctionsState::on_quit() {
}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("points_and_functions", PointsAndFunctionsState, 50ms)
