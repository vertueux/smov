#include "points_and_functions.h"
  
void PointsAndFunctionsState::on_start() {
  for (int i = 0; i < 6; i++) {
    front_servos.value[i] = 0.0f;
  }

  front_state_publisher->publish(front_servos);

  smov::TrigonometryState trig = smov::TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &leg_width, &hip_body_distance);

  front_servos.value[1] = 90.0f;
  front_servos.value[3] = 55.0f;
  front_servos.value[5] = 30.0f; // (val - base) / (max - base) = -1

  smov::Vector3 coord(0, 12, 12);
  trig.set_leg_to(1, coord);

  for (int i = 0; i < 6; i++) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Values: %f", front_servos.value[i]);
  }
}

void PointsAndFunctionsState::on_loop() {
}

void PointsAndFunctionsState::on_quit() {
}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("points_and_functions", PointsAndFunctionsState, 500ms)
