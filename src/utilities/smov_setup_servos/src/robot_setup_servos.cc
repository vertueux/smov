#include <iostream>
#include <memory>

#include <setup_servos/robot_setup_servos.h>

using namespace std::chrono_literals;

namespace smov {

ServosSetup::ServosSetup() : Node("smov_setup_servos") {
  // Declaring the different parameters.
  declare_parameters();

  // Setting up the publishers.
  set_up_topics();

  // Configuring the proportional servos with their defined values.
  config_servos();
}

void ServosSetup::declare_parameters() {
  std::vector<long int> default_value(8, 0); // 8: number of elements in the .yaml array.

  // Declaring all default parameters first.
  for (size_t j = 0; j < servo_name.size(); j++)
    this->declare_parameter(servo_name[j], default_value);

  // Declaring the default values.
  this->declare_parameter("use_single_board", use_single_board);

  // Getting the new parameters from the user.
  use_single_board = this->get_parameter("use_single_board").as_bool();

  // We initialize the arrays with their default values.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_servos_data.push_back(this->get_parameter(servo_name[i]).as_integer_array());
    back_servos_data.push_back(this->get_parameter(servo_name[i + SERVO_MAX_SIZE]).as_integer_array());
  }
}

void ServosSetup::set_up_topics() {
  // Setting up the servo config client.
  front_servo_config_client = this->create_client<i2c_pwm_board_msgs::srv::ServosConfig>("main_config_servos");
  if (!use_single_board)
    back_servo_config_client = this->create_client<i2c_pwm_board_msgs::srv::ServosConfig>("secondary_config_servos");

  RCLCPP_INFO(this->get_logger(), "Set up /config_servos_handler publishers.");
}

void ServosSetup::config_servos() {
  i2c_pwm_board_msgs::msg::ServoConfig front_config[SERVO_MAX_SIZE * 2];
  i2c_pwm_board_msgs::msg::ServoConfig back_config[SERVO_MAX_SIZE];

  auto front_request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();
  auto back_request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();

  for (int h = 0; h < SERVO_MAX_SIZE; h++) {
    front_config[h].servo = static_cast<short>(front_servos_data[h][0] + 1);
    front_config[h].direction = static_cast<short>(front_servos_data[h][3]);
    front_config[h].center = static_cast<short>(front_servos_data[h][1]);
    front_config[h].range = static_cast<short>(front_servos_data[h][2]);
    front_request->servos.push_back(front_config[h]);

    if (use_single_board) {
      front_config[h + SERVO_MAX_SIZE].range = static_cast<short>(back_servos_data[h][2]);
      front_config[h + SERVO_MAX_SIZE].direction = static_cast<short>(back_servos_data[h][3]);
      front_config[h + SERVO_MAX_SIZE].center = static_cast<short>(back_servos_data[h][1]);
      front_config[h + SERVO_MAX_SIZE].servo = static_cast<short>(back_servos_data[h][0] + 1);
      front_request->servos.push_back(front_config[h + SERVO_MAX_SIZE]);
    } else {
      back_config[h].range = static_cast<short>(back_servos_data[h][2]);
      back_config[h].direction = static_cast<short>(back_servos_data[h][3]);
      back_config[h].center = static_cast<short>(back_servos_data[h][1]);
      back_config[h].servo = static_cast<short>(back_servos_data[h][0] + 1);
      back_request->servos.push_back(back_config[h]);
    }
  }

  while (!front_servo_config_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the Front Stop Servos service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Front Stop Servos service not available, waiting again...");
  }
  if (!use_single_board) {
    while (!back_servo_config_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Back Stop Servos service not available, waiting again...");
    }
  }
  auto f_result = front_servo_config_client->async_send_request(front_request);
  if (!use_single_board) auto b_result = back_servo_config_client->async_send_request(back_request);

  RCLCPP_INFO(this->get_logger(), "Servos have been configured.");
}

} // namespace smov

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::ServosSetup>());
  rclcpp::shutdown();
  return 0;
}
