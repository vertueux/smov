//
// Created by ros on 2/3/24.
//

#include "board_handler.h"

namespace smov {

BoardHandler::BoardHandler(const std::string &node_name) {
  this->board_node = std::make_shared<smov::BoardNode>(node_name);
}

void BoardHandler::init(int io_device, int frequency) {
  this->board_node->init(io_device, frequency);
}

void BoardHandler::set_handlers(int board_number) {

  if (board_number == 1) {

    this->config_srv = this->board_node->create_service<smov_board_msgs::srv::ServosConfig>("front_config_servos",
                                                                                       std::bind(&BoardHandler::config_servos_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2));     // 'config' will setup the necessary properties of continuous servos and is helpful for standard servos.
    this->abs_sub = this->board_node->create_subscription<smov_board_msgs::msg::ServoArray>("front_servos_absolute",
                                                                                       500,
                                                                                       std::bind(&BoardHandler::servos_absolute_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1));             // The 'absolute' topic will be used for standard servo motion and testing of continuous servos.

    this->rel_sub = this->board_node->create_subscription<smov_board_msgs::msg::ServoArray>("front_servos_proportional",
                                                                                       500,
                                                                                       std::bind(&BoardHandler::servos_proportional_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1)); // The 'proportion' topic will be used for standard servos and continuous rotation aka drive servos.

    this->freq_srv = this->board_node->create_service<smov_board_msgs::srv::IntValue>("front_set_pwm_frequency",
                                                                                 std::bind(&BoardHandler::set_pwm_frequency_handler,
                                                                                           this,
                                                                                           std::placeholders::_1,
                                                                                           std::placeholders::_2));
    this->mode_srv = this->board_node->create_service<smov_board_msgs::srv::DriveMode>("front_config_drive_mode",
                                                                                  std::bind(&BoardHandler::config_drive_mode_handler,
                                                                                            this,
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2));                    // 'mode' specifies which servos are used for motion and which behavior will be applied when driving.
    this->stop_srv = this->board_node->create_service<std_srvs::srv::Empty>("front_stop_servos",
                                                                            std::bind(&BoardHandler::stop_servos_handler,
                                                                                      this,
                                                                                      std::placeholders::_1,
                                                                                      std::placeholders::_2));                                         // The 'stop' service can be used at any time.
    this->drive_sub = this->board_node->create_subscription<geometry_msgs::msg::Twist>("front_servos_drive",
                                                                                       500,
                                                                                       std::bind(&BoardHandler::servos_drive_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1));                     // The 'drive' topic will be used for continuous rotation aka drive servos controlled by Twist messages.
  } else {
    this->config_srv = this->board_node->create_service<smov_board_msgs::srv::ServosConfig>("back_config_servos",
                                                                                       std::bind(&BoardHandler::config_servos_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2));
    this->abs_sub = this->board_node->create_subscription<smov_board_msgs::msg::ServoArray>("back_servos_absolute",
                                                                                       500,
                                                                                       std::bind(&BoardHandler::servos_absolute_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1));
    this->rel_sub = this->board_node->create_subscription<smov_board_msgs::msg::ServoArray>("back_servos_proportional",
                                                                                       500,
                                                                                       std::bind(&BoardHandler::servos_proportional_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1));

    this->freq_srv = this->board_node->create_service<smov_board_msgs::srv::IntValue>("back_set_pwm_frequency",
                                                                                 std::bind(&BoardHandler::set_pwm_frequency_handler,
                                                                                           this,
                                                                                           std::placeholders::_1,
                                                                                           std::placeholders::_2));

    this->mode_srv = this->board_node->create_service<smov_board_msgs::srv::DriveMode>("back_config_drive_mode",
                                                                                  std::bind(&BoardHandler::config_drive_mode_handler,
                                                                                            this,
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2));

    this->stop_srv = this->board_node->create_service<std_srvs::srv::Empty>("back_stop_servos",
                                                                            std::bind(&BoardHandler::stop_servos_handler,
                                                                                      this,
                                                                                      std::placeholders::_1,
                                                                                      std::placeholders::_2));

    this->drive_sub = this->board_node->create_subscription<geometry_msgs::msg::Twist>("back_servos_drive",
                                                                                       500,
                                                                                       std::bind(&BoardHandler::servos_drive_handler,
                                                                                                 this,
                                                                                                 std::placeholders::_1));
  }
}

void BoardHandler::servos_absolute_handler(const std::shared_ptr<smov_board_msgs::msg::ServoArray> msg) {
  for (auto &sp : msg->servos) {
    int servo = sp.servo;
    int value = sp.value;

    if ((value < 0) || (value > 4096)) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Invalid PWM value %d :: PWM values must be between 0 and 4096",
                   value);
      continue;
    }
    this->board_node->set_pwm_interval(servo, 0, value);
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "servo[%d] = %d", servo, value);
  }
}

void BoardHandler::servos_proportional_handler(const std::shared_ptr<smov_board_msgs::msg::ServoArray> msg) {
  for (auto &sp : msg->servos) {
    int servo = sp.servo;
    float value = sp.value;
    this->board_node->set_pwm_interval_proportional(servo, value);
  }
}

void BoardHandler::servos_drive_handler(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
  int i;
  float delta, range, ratio;
  float temp_x, temp_y, temp_r;
  float dir_x, dir_y, dir_r;
  float speed[4];

  /* "msg" is a pointer to a Twist message: msg->linear and msg->angular each of which have members .x .y .z .*/
  /* the subscriber uses the maths from: http://robotsforroboticists.com/drive-kinematics/ . */

  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "servos_drive_handler Twist = [%5.2f %5.2f %5.2f] [%5.2f %5.2f %5.2f]",
               msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

  if (this->board_node->get_active_drive().mode == smov::MODE_UNDEFINED) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "drive mode not set");
    return;
  }
  if ((this->board_node->get_active_drive().mode < smov::MODE_UNDEFINED)
      || (this->board_node->get_active_drive().mode >= smov::MODE_INVALID)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "unrecognized drive mode set %d",
                 this->board_node->get_active_drive().mode);
    return;
  }

  dir_x = static_cast<float>(((msg->linear.x < 0) ? -1 : 1));
  dir_y = static_cast<float>(((msg->linear.y < 0) ? -1 : 1));
  dir_r = static_cast<float>(((msg->angular.z < 0) ? -1 : 1));

  temp_x = static_cast<float>(this->board_node->get_active_drive().scale * std::abs(msg->linear.x));
  temp_y = static_cast<float>(this->board_node->get_active_drive().scale * std::abs(msg->linear.y));
  temp_r = static_cast<float>(std::abs(msg->angular.z));    // Radians.

  // The differential rate is the robot rotational circumference / angular velocity.
  // Since the differential rate is applied to both sides in opposite amounts it is halved.
  delta = (this->board_node->get_active_drive().track / 2) * temp_r;
  // Delta is now in meters/sec.

  // Determine if we will over-speed the motor and scale accordingly.
  ratio = this->board_node->convert_mps_to_proportional(temp_x + delta);
  if (ratio > 1.0)
    temp_x /= ratio;

  switch (this->board_node->get_active_drive().mode) {
    case smov::MODE_ACKERMAN:
      /*
        With ackerman drive, steering is handled by a separate servo.
        We drive assigned servos exclusively by the linear.x.
      */
      speed[0] = temp_x * dir_x;
      speed[0] = this->board_node->convert_mps_to_proportional(speed[0]);
      if (std::abs(speed[0]) > 1.0)
        speed[0] = static_cast<float>(1.0 * dir_x);

      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "ackerman drive mode speed=%6.4f", speed[0]);
      break;
    case smov::MODE_DIFFERENTIAL:
      /*
        With differential drive, steering is handled by the relative speed of left and right servos.
        We drive assigned servos by mixing linear.x and angular.z.
        We compute the delta for left and right components.
        We use the sign of the angular velocity to determine which is the faster / slower.
      */

      /* The delta is the angular velocity * half the drive track. */

      if (dir_r > 0) {    // Turning right.
        speed[0] = (temp_x + delta) * dir_x;
        speed[1] = (temp_x - delta) * dir_x;
      } else {        // Turning left.
        speed[0] = (temp_x - delta) * dir_x;
        speed[1] = (temp_x + delta) * dir_x;
      }

      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "computed differential drive mode speed left=%6.4f right=%6.4f",
                   speed[0],
                   speed[1]);

      /* Ff any of the results are greater that 1.0, we need to scale all the results down. */
      range = std::max(std::abs(speed[0]), std::abs(speed[1]));

      ratio = this->board_node->convert_mps_to_proportional(range);
      if (ratio > 1.0) {
        speed[0] /= ratio;
        speed[1] /= ratio;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "adjusted differential drive mode speed left=%6.4f right=%6.4f",
                   speed[0],
                   speed[1]);

      speed[0] = this->board_node->convert_mps_to_proportional(speed[0]);
      speed[1] = this->board_node->convert_mps_to_proportional(speed[1]);

      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "differential drive mode speed left=%6.4f right=%6.4f",
                   speed[0],
                   speed[1]);
      break;
    case smov::MODE_MECANUM:
      /*
        With mecanum drive, steering is handled by the relative speed of left and right servos.
        With mecanum drive, lateral motion is handled by the rotation of front and rear servos.
        We drive assigned servos by mixing linear.x and angular.z  and linear.y.
      */

      if (dir_r > 0) {    // Turning right.
        speed[0] = speed[2] = (temp_x + delta) * dir_x;
        speed[1] = speed[3] = (temp_x - delta) * dir_x;
      } else {        // Turning left.
        speed[0] = speed[2] = (temp_x - delta) * dir_x;
        speed[1] = speed[3] = (temp_x + delta) * dir_x;
      }

      speed[0] += temp_y * dir_y;
      speed[3] += temp_y * dir_y;
      speed[1] -= temp_y * dir_y;
      speed[2] -= temp_y * dir_y;
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "computed mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f",
                   speed[0],
                   speed[1],
                   speed[2],
                   speed[3]);

      range =
          std::max(std::max(std::max(std::abs(speed[0]), std::abs(speed[1])), std::abs(speed[2])), std::abs(speed[3]));
      ratio = this->board_node->convert_mps_to_proportional(range);
      if (ratio > 1.0) {
        speed[0] /= ratio;
        speed[1] /= ratio;
        speed[2] /= ratio;
        speed[3] /= ratio;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "adjusted mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f",
                   speed[0],
                   speed[1],
                   speed[2],
                   speed[3]);

      speed[0] = this->board_node->convert_mps_to_proportional(speed[0]);
      speed[1] = this->board_node->convert_mps_to_proportional(speed[1]);
      speed[2] = this->board_node->convert_mps_to_proportional(speed[2]);
      speed[3] = this->board_node->convert_mps_to_proportional(speed[3]);

      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),
                   "mecanum drive mode speed leftfront=%6.4f rightfront=%6.4f leftrear=%6.4f rightreer=%6.4f",
                   speed[0],
                   speed[1],
                   speed[2],
                   speed[3]);
      break;
    default:break;
  }

  /* Find all drive servos and set their new speed. */
  for (i = 0; i < (this->board_node->get_last_servo()); i++) {
    // We use 'fall through' on the switch statement to allow all necessary servos to be controlled.
    switch (this->board_node->get_active_drive().mode) {
      case smov::MODE_MECANUM:
        if (this->board_node->servo_configs[i].mode_pos == smov::POSITION_RIGHTREAR)
          this->board_node->set_pwm_interval_proportional(i + 1, speed[3]);
        if (this->board_node->servo_configs[i].mode_pos == smov::POSITION_LEFTREAR)
          this->board_node->set_pwm_interval_proportional(i + 1, speed[2]);
        break; // This may cause an issue.   ^
        //                            |
      case smov::MODE_DIFFERENTIAL:
        if (this->board_node->servo_configs[i].mode_pos == smov::POSITION_RIGHTFRONT)
          this->board_node->set_pwm_interval_proportional(i + 1, speed[1]);
        break; // This may cause an issue.
      case smov::MODE_ACKERMAN:
        if (this->board_node->servo_configs[i].mode_pos == smov::POSITION_LEFTFRONT)
          this->board_node->set_pwm_interval_proportional(i + 1, speed[0]);
        break; // This may cause an issue.
    }
  }
}

bool BoardHandler::set_pwm_frequency_handler(const std::shared_ptr<smov_board_msgs::srv::IntValue::Request> req,
                                             std::shared_ptr<smov_board_msgs::srv::IntValue::Response> res) {
  int freq;
  freq = req->value;
  if ((freq < 12) || (freq > 1024)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Invalid PWM frequency %d :: PWM frequencies should be between 12 and 1024",
                 freq);
    freq = 50;    // Most analog RC servos are designed for 20ms pulses.
    res->error = static_cast<int16_t>(freq);
  }

  this->board_node->set_pwm_frequency(freq);    // I think we must reset frequency when we change boards.
  res->error = static_cast<int16_t>(freq);
  return true;
}

bool BoardHandler::config_servos_handler(const std::shared_ptr<smov_board_msgs::srv::ServosConfig::Request> req,
                                         std::shared_ptr<smov_board_msgs::srv::ServosConfig::Response> res) {
  long unsigned int i;

  res->error = 0;

  if ((this->board_node->get_active_board() < 1) || (this->board_node->get_active_board() > 62)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Internal error - invalid board number %d :: PWM board numbers must be between 1 and 62",
                 this->board_node->get_active_board());
    res->error = -1;
    return true;
  }

  for (i = 0; i < req->servos.size(); i++) {
    int servo = req->servos[i].servo;
    int center = req->servos[i].center;
    int range = req->servos[i].range;
    int direction = req->servos[i].direction;

    this->board_node->config_servo(servo, center, range, direction);

  }

  return true;
}

bool BoardHandler::config_drive_mode_handler(const std::shared_ptr<smov_board_msgs::srv::DriveMode::Request> req,
                                             std::shared_ptr<smov_board_msgs::srv::DriveMode::Response> res) {
  res->error = 0;
  long unsigned int i;

  if ((res->error = this->board_node->config_drive_mode(req->mode, req->rpm, req->radius, req->track, req->scale)))
    return true;

  for (i = 0; i < req->servos.size(); i++) {
    int servo = req->servos[i].servo;
    int position = req->servos[i].position;

    if (this->board_node->config_servo_position(servo, position) != 0) {
      res->error = servo; /* this needs to be more specific and indicate a bad server ID was provided */
      continue;
    }
  }

  return true;
}

bool BoardHandler::stop_servos_handler(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                       std::shared_ptr<std_srvs::srv::Empty::Response> res) {

  // In order to remove a compiler warning.
  UNUSED(req);
  UNUSED(res);

  int save_active = this->board_node->get_active_board();
  int i = 0;

  for (i = 0; i < MAX_BOARDS; i++) {
    if (this->board_node->pwm_boards[i] > 0) {
      this->board_node->set_active_board(i + 1);    // API is ONE based.
      this->board_node->set_pwm_interval_all(0, 0);
    }
  }
  this->board_node->set_active_board(save_active);
  return true;
}

}