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
  coord.x = 15;
  coord.y = 15;
  coord.z = 10;
  trig.set_leg_to(1, coord);
}

void PointsAndFunctionsState::on_loop() {
  // Creating a function: f(x) = (10/16)sqrt(25-(x-10)^2) + 15
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
  coord.y = (10.0f / 16.0f) * sqrt(25.0f - pow((coord.x - 10.0f), 2)) + 15.0f;
  if (coord.x >= 5)
    coord.x -= 0.4f;
  else {
    coord.x = 15.0f;
    coord.y = 15.0f;
  }
  trig.set_leg_to(1, coord);
}

void PointsAndFunctionsState::on_quit() {
}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("points_and_functions", PointsAndFunctionsState, 150ms)
