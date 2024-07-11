#include "walking_gait.h"

float ForwardMotion::curved(float x, float gap) {
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
  coord3.y = 23 + back_leg_gap;
  coord3.z = 5;

  coord4.x = 3.5f;
  coord4.y = 23 + back_leg_gap;
  coord4.z = 5;

  trig.set_leg_to(1, coord1);
  trig.set_leg_to(2, coord2);
  trig.set_leg_to(3, coord3);
  trig.set_leg_to(4, coord4);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set default position to (3.5, 23-24, 5)");
}

void ForwardMotion::output_coordinates() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates leg 1: (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates leg 2: (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates leg 3: (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates leg 4: (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
  if (mode == STANDING) 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot Mode:        STANDING");
  else if (mode == WALKING) 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot Mode:        WALKING");
  else 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot Mode:        TURNING");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done leg 1: %d", leg1_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done leg 2: %d", leg2_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done leg 3: %d", leg3_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motion done leg 4: %d", leg4_motion_done);
}

void ForwardMotion::walk() {
  if (!leg1_motion_done) {
    if (coord1.x > -1.45f) {
      coord1.x = smov::Functions::lerp(coord1.x, -1.5f, 0.15f);
      coord1.y = curved(coord1.x, 0.0f);
      trig.set_leg_to(1, coord1);
    } else {
      leg1_motion_done = true;
      if (!request_to_stop_walk) leg2_motion_done = false;
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
      coord4.y = curved(coord4.x, back_leg_gap);
      trig.set_leg_to(4, coord4);
    } else {
      leg4_motion_done = true;
      if (!request_to_stop_walk) leg3_motion_done = false;
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
      if (!request_to_stop_walk) leg1_motion_done = false;
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
      coord3.y = curved(coord3.x, back_leg_gap);
      trig.set_leg_to(3, coord3);
    } else {
      leg3_motion_done = true;
      if (!request_to_stop_walk) leg4_motion_done = false;
    }
  } else {
    if (coord3.x < 3.45f) {
      coord3.x = smov::Functions::lerp(coord3.x, 3.5f, 0.15f);
      trig.set_leg_to(3, coord3);
    }
  }
}

void ForwardMotion::turn_right() {

}

void ForwardMotion::on_start() {
  // Getting the default config.
  tcgetattr(0, &old_chars);

  // Initializing the reader.
  fcntl(0, F_SETFL, O_NONBLOCK);
  new_chars = old_chars;
  new_chars.c_lflag &= ~ICANON;
  new_chars.c_lflag &= 0 ? ECHO : ~ECHO; // echo = 0.
  tcsetattr(0, TCSANOW, &new_chars);

  stabilize_legs();
}

void ForwardMotion::on_loop() {
  int c = getchar();
  switch (c) {
    case 65: // 65: Key up.
      if (mode == STANDING) {
        mode = WALKING;
        request_to_stop_walk = false;
      }
      break;
    case 66: // 66: Key down. 
      if (has_finished_walk) 
        mode = STANDING;
      else 
        request_to_stop_walk = true;
      break;
    case 67: // 67: Key right.
      request_to_stop_walk = false;
      if (mode == STANDING) mode = TURNING;
      break;
  }

  output_coordinates();

  if (smov::Functions::approx(coord1.x, 3.5f, 0.06f) && smov::Functions::approx(coord2.x, 3.5f, 0.06f) 
    && smov::Functions::approx(coord3.x, 3.5f, 0.06f) && smov::Functions::approx(coord4.x, 3.5f, 0.06f) && request_to_stop_walk) {
    has_finished_walk = true;
    mode = STANDING;
    if (mode == STANDING) done_once = false;
  } else 
    has_finished_walk = false;

  if (mode == WALKING) {
    if (done_once == false) {
      // Some code that executes only once.
      leg1_motion_done = false;
      leg4_motion_done = false;
      done_once = true;
    }
    walk();
  }

  if (mode == TURNING) {
    turn_right();
  }
}

void ForwardMotion::on_quit() {
  // Changing to default config.
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

DECLARE_STATE_NODE_CLASS("walking_gait", ForwardMotion, 50ms)
