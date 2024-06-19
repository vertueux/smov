#include "walking_gait.h"

float ForwardMotion::curved(float x, float gap) {
  // Gap should always be equal to 0 if servos have been centered perfectly.
  return -sqrt(25.0f - pow((2 * x - 2.0f), 2)) + 23.0f + gap;
}

void ForwardMotion::stabilize_legs() {
  coord1.x = 3.5f;
  coord1.y = 23;
  coord1.z = 5;

  coord2.x = 3.5f;
  coord2.y = 23;
  coord2.z = 5;

  coord3.x = 3.5f;
  coord3.y = 24;
  coord3.z = 5;

  coord4.x = 3.5f;
  coord4.y = 24;
  coord4.z = 5;

  trig.set_leg_to(1, coord1);
  trig.set_leg_to(2, coord2);
  trig.set_leg_to(3, coord3);
  trig.set_leg_to(4, coord4);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set default position to (3.5, 23-24, 5)");
}

void ForwardMotion::output_coordinates() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates 1: (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates 2: (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates 3: (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates 4: (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done 1: %d", leg1_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done 2: %d", leg2_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done 3: %d", leg3_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done 4: %d", leg4_motion_done);
}

void ForwardMotion::on_start() {
  stabilize_legs();
}

void ForwardMotion::on_loop() {
  output_coordinates();

  if (!leg1_motion_done) {
    if (coord1.x > -1.45f) {
      coord1.x = smov::Functions::lerp(coord1.x, -1.5f, 0.15f);
      coord1.y = curved(coord1.x, 0.0f);
      trig.set_leg_to(1, coord1);
    } else {
      leg1_motion_done = true;
      leg2_motion_done = false;
    }
  } else {
    if (coord1.x < 3.45f) {
      coord1.x = smov::Functions::lerp(coord1.x, 3.5f, 0.15f);
      trig.set_leg_to(1, coord1);
    }
  }

  if (!leg4_motion_done) {
    if (coord4.x > -1.45f) {
      coord4.x = smov::Functions::lerp(coord4.x, -1.5f, 0.15f);
      coord4.y = curved(coord4.x, 4.5f);
      trig.set_leg_to(4, coord4);
    } else {
      leg4_motion_done = true;
      leg3_motion_done = false;
    }
  } else {
    if (coord4.x < 3.45f) {
      coord4.x = smov::Functions::lerp(coord4.x, 3.5f, 0.15f);
      trig.set_leg_to(4, coord4);
    }
  }

  if (!leg2_motion_done) {
    if (coord2.x > -1.45f) {
      coord2.x = smov::Functions::lerp(coord2.x, -1.5f, 0.15f);
      coord2.y = curved(coord2.x, 0.0f);
      trig.set_leg_to(2, coord2);
    } else {
      leg2_motion_done = true;
      leg1_motion_done = false;
    }
  } else {
    if (coord2.x < 3.45f) {
      coord2.x = smov::Functions::lerp(coord2.x, 3.5f, 0.15f);
      trig.set_leg_to(2, coord2);
    }
  }

  if (!leg3_motion_done) {
    if (coord3.x > -1.45f) {
      coord3.x = smov::Functions::lerp(coord3.x, -1.5f, 0.15f);
      coord3.y = curved(coord3.x, 3.5f);
      trig.set_leg_to(3, coord3);
    } else {
      leg3_motion_done = true;
      leg4_motion_done = false;
    }
  } else {
    if (coord3.x < 3.45f) {
      coord3.x = smov::Functions::lerp(coord3.x, 3.5f, 0.15f);
      trig.set_leg_to(3, coord3);
    }
  }
}

void ForwardMotion::on_quit() {
    
}

DECLARE_STATE_NODE_CLASS("walking_gait", ForwardMotion, 50ms)
