#include "configure_servos.h"

namespace smov {

ConfigServos::ConfigServos() : Node("smov_config_servos") {
  // Declaring the different parameters.
  declare_parameters();

  // Setting up the publishers.
  set_up_topics();

  // Configuring the proportional servos with their defined values.
  config_servos();

  // Then exiting the program.
  rclcpp::shutdown();
}

void ConfigServos::declare_parameters() {
  std::vector<long int> default_value(8, 0); // 8: number of elements in the .yaml array.

  // Declaring all default parameters first.
  for (size_t j = 0; j < servo_name.size(); j++)
    this->declare_parameter(servo_name[j], default_value);

  // Declaring the default values.
  this->declare_parameter("use_single_board", use_single_board);
  this->declare_parameter("front_board_i2c_bus", front_board_i2c_bus);
  this->declare_parameter("back_board_i2c_bus", back_board_i2c_bus);

  // Getting the new parameters from the user.
  use_single_board = this->get_parameter("use_single_board").as_bool();
  front_board_i2c_bus = this->get_parameter("front_board_i2c_bus").as_int();
  if (!use_single_board) back_board_i2c_bus = this->get_parameter("back_board_i2c_bus").as_int();

  // We initialize the arrays with their default values.
  for (int i = 0; i < 6; i++) {
    front_servos_data.push_back(this->get_parameter(servo_name[i]).as_integer_array());
    back_servos_data.push_back(this->get_parameter(servo_name[i + 6]).as_integer_array());
  }
}

void ConfigServos::set_up_topics() {
  // Setting up the servo config client.
  front_servo_config_client = this->create_client<i2c_pwm_board_msgs::srv::ServosConfig>("config_servos_" + std::to_string(front_board_i2c_bus));
  if (!use_single_board)
    back_servo_config_client = this->create_client<i2c_pwm_board_msgs::srv::ServosConfig>("config_servos_" + std::to_string(back_board_i2c_bus));

  RCLCPP_INFO(this->get_logger(), "Set up /config_servos_handler publishers.");
}

void ConfigServos::config_servos() {
  if (use_single_board) {
    auto front_request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();
    i2c_pwm_board_msgs::msg::ServoConfig front_config[6 * 2];
    for (int h = 0; h < 6; h++) {
      front_config[h].servo = static_cast<short>(front_servos_data[h][0] + 1);
      front_config[h].direction = static_cast<short>(front_servos_data[h][3]);
      front_config[h].center = static_cast<short>(front_servos_data[h][1]);
      front_config[h].range = static_cast<short>(front_servos_data[h][2]);
      front_config[h + 6].range = static_cast<short>(back_servos_data[h][2]);
      front_config[h + 6].direction = static_cast<short>(back_servos_data[h][3]);
      front_config[h + 6].center = static_cast<short>(back_servos_data[h][1]);
      front_config[h + 6].servo = static_cast<short>(back_servos_data[h][0] + 1);
      front_request->servos.push_back(front_config[h]);
      front_request->servos.push_back(front_config[h + 6]);
    } 
    while (!front_servo_config_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the Front Config Servos service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Front Config Servos service not available, waiting again...");
      auto f_result = front_servo_config_client->async_send_request(front_request);
    }
  } else {
    auto front_request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();
    auto back_request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();
    i2c_pwm_board_msgs::msg::ServoConfig front_config[6];
    i2c_pwm_board_msgs::msg::ServoConfig back_config[6];
    for (int h = 0; h < 6; h++) {
      front_config[h].servo = static_cast<short>(front_servos_data[h][0] + 1);
      front_config[h].direction = static_cast<short>(front_servos_data[h][3]);
      front_config[h].center = static_cast<short>(front_servos_data[h][1]);
      front_config[h].range = static_cast<short>(front_servos_data[h][2]);
      back_config[h].range = static_cast<short>(back_servos_data[h][2]);
      back_config[h].direction = static_cast<short>(back_servos_data[h][3]);
      back_config[h].center = static_cast<short>(back_servos_data[h][1]);
      back_config[h].servo = static_cast<short>(back_servos_data[h][0] + 1);
      front_request->servos.push_back(front_config[h]);
      back_request->servos.push_back(back_config[h]);
    }
    while (!back_servo_config_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the Back Config Servos service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Back Config Servos service not available, waiting again...");
    }
    auto b_result = back_servo_config_client->async_send_request(back_request);
  }
  RCLCPP_INFO(this->get_logger(), "Servos have been configured.");
}

} // namespace smov

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::ConfigServos>());
  return 0;
}
