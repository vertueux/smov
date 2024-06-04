#include <math.h>

#include "walking_simulation.h"

void WalkingSimulationState::initialize_coordinates() {
  coord1.x = 14;
  coord1.y = 15;
  coord1.z = 5;
  trig.set_leg_to(1, coord1);

  coord2.x = 14;
  coord2.y = 15;
  coord2.z = 5;
  trig.set_leg_to(2, coord2);

  coord3.x = 14;
  coord3.y = 15;
  coord3.z = 5;
  trig.set_leg_to(3, coord3);

  coord4.x = 14;
  coord4.y = 15;
  coord4.z = 5;
  trig.set_leg_to(4, coord4);

}

void WalkingSimulationState::on_start() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting default position to (15, 15, 10)");
  initialize_coordinates();
}

bool done_first_step = false;

void WalkingSimulationState::on_loop() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "coordinates 1: (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "coordinates 2: (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "coordinates 3: (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "coordinates 4: (%f, %f, %f)", coord4.x, coord4.y, coord4.z);

  // Creating a function: f(x) = -sqrt(25 - ((coord1.x / 2) - 5)^2) + 20.
  if (!done_first_step && (coord1.x >= -6 || coord4.x >= -6)) {
    coord1.y = -sqrt(25.0f - pow(((coord1.x / 2) - 2.0f), 2)) + 15.0f; // We have to multiply by -1 to turn upside down the curve to match.
    coord4.y = -sqrt(25.0f - pow(((coord4.x / 2) - 2.0f), 2)) + 15.0f; // We have to multiply by -1 to turn upside down the curve to match.

    coord1.x -= 1.0f;
    coord4.x -= 1.0f;
    trig.set_leg_to(1, coord1);
    trig.set_leg_to(4, coord4);
  }
  else if (!done_first_step) {
    coord1.x = 14.0f;
    coord1.y = 15.0f;
    coord4.x = 14.0f;
    coord4.y = 15.0f;
    trig.set_leg_to(1, coord1);
    trig.set_leg_to(4, coord4);
    done_first_step = true;
  }

  if (done_first_step && (coord2.x >= -6 || coord3.x >= -6)) {
    coord2.y = -sqrt(25.0f - pow(((coord2.x / 2) - 2.0f), 2)) + 15.0f; // We have to multiply by -1 to turn upside down the curve to match.
    coord3.y = -sqrt(25.0f - pow(((coord3.x / 2) - 2.0f), 2)) + 15.0f; // We have to multiply by -1 to turn upside down the curve to match.

    coord2.x -= 1.0f;
    coord3.x -= 1.0f;
    trig.set_leg_to(2, coord2);
    trig.set_leg_to(3, coord3);
  } else if (done_first_step) {
    coord2.x = 14.0f;
    coord2.y = 15.0f;
    coord3.x = 14.0f;
    coord3.y = 15.0f;
    trig.set_leg_to(2, coord2);
    trig.set_leg_to(3, coord3);
    done_first_step = false;
  }
}

void WalkingSimulationState::on_quit() {
}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("walking_simulation", WalkingSimulationState, 50ms)
