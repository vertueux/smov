#include <iostream>
#include <memory>

#include <servos_setup/robot_servos_setup.h>

using namespace std::chrono_literals;

namespace smov {

ServosSetup::ServosSetup()
    : Node("smov_states") {

  // Declaring the different parameters.
  declare_parameters();

  // Locking the servos on start.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempt to lock the servos at their initial value.");

  // Setting up the publishers.
  set_up_topics();

  // Configuring the proportional servos with their defined values.
  config_servos();
}

void ServosSetup::declare_parameters() {
  std::vector<long int> default_value(5, 0);

  // Declaring all default parameters first.
  for (const auto &j : servo_name) {
    this->declare_parameter(j, default_value);
  }

  // Declaring the board option.
  this->declare_parameter("use_single_board", false);

  // Initializing it already to prevent a warning / error during the launch.
  use_single_board = this->get_parameter("use_single_board").as_bool();

  // We initialize the arrays with their default values.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) { // 5 is the number of data in a single array (in ~/parameters.yaml).
    front_servos_data.push_back(this->get_parameter(servo_name[i]).as_integer_array());
    back_servos_data.push_back(this->get_parameter(servo_name[i + SERVO_MAX_SIZE]).as_integer_array());
  }
}

void ServosSetup::set_up_topics() {
  // Setting up the servo config client.
  front_servo_config_client = this->create_client<smov_board_msgs::srv::ServosConfig>("front_config_servos");
  if (!use_single_board)
    back_servo_config_client = this->create_client<smov_board_msgs::srv::ServosConfig>("back_config_servos");

  RCLCPP_INFO(this->get_logger(), "Set up /config_servos_handler publisher.");
}

void ServosSetup::config_servos() {
  smov_board_msgs::msg::ServoConfig front_config[SERVO_MAX_SIZE * 2];
  smov_board_msgs::msg::ServoConfig back_config[SERVO_MAX_SIZE];

  auto front_request = std::make_shared<smov_board_msgs::srv::ServosConfig::Request>();
  auto back_request = std::make_shared<smov_board_msgs::srv::ServosConfig::Request>();

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

  while (!front_servo_config_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
  if (!use_single_board) {
    while (!back_servo_config_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
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
