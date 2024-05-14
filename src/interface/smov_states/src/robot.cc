#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include <states/robot.h>

namespace smov {

RobotData::RobotData() = default;
RobotData::~RobotData() = default;

// Initializing default static values.
RobotData *RobotData::instance = nullptr;
RobotData *RobotData::Instance() {
  if (!instance)
    instance = new RobotData;
  return instance;
}

bool RobotData::use_single_board = false;
double RobotData::upper_leg_length = 14.0;
double RobotData::lower_leg_length = 14.0;
double RobotData::hip_body_distance = 4.0;

RobotNodeHandle::RobotNodeHandle() : Node("smov_states") {
  // Declaring the different parameters.
  declare_parameters();

  // Locking the servos on start.
  RCLCPP_INFO(this->get_logger(), "Attempt to lock the servos at their initial value.");

  // Default configuration.
  set_up_servos();

  // Setting up the publishers.
  set_up_topics();

  // Setting the display lines.
  robot->up_display.line = 1;

  // Configuring the proportional servos with their defined values.
  config_servos();

  // Publishing the proportional values.
  front_prop_pub->publish(robot->front_prop_array);
  if (robot->use_single_board)
    front_prop_pub->publish(robot->single_back_array);
  else
    back_prop_pub->publish(robot->back_prop_array);

  // Calling the loops with some timeouts.
  late_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RobotNodeHandle::output_values, this));
}

void RobotNodeHandle::front_topic_callback(smov_states_msgs::msg::StatesServos::SharedPtr msg) {
  if (robot->state == "None") {
    robot->state = msg->state_name.c_str();
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Detecting a new state: %s", robot->state.c_str());
    RCLCPP_INFO(this->get_logger(), "===========================================");
  }

  // IMPORTANT CHANGE: We now changed the value to only be angles (which are then converted into proportional values).
  if (msg->state_name == robot->state) {
    for (int i = 0; i < SERVO_MAX_SIZE; i++) {
      if (msg->value[i] < (2 * robot->front_servos_data[i][5]) - robot->front_servos_data[i][6]) {
        RCLCPP_ERROR(this->get_logger(), "===========================================");
        RCLCPP_ERROR(this->get_logger(), "Angle out of range (minimum)");
        RCLCPP_ERROR(this->get_logger(), "===========================================");
      } else if (msg->value[i] > robot->front_servos_data[i][6]) {
        RCLCPP_ERROR(this->get_logger(), "===========================================");
        RCLCPP_ERROR(this->get_logger(), "Angle out of range (maximum)");
        RCLCPP_ERROR(this->get_logger(), "===========================================");
      } else {

        // As the numerical value is between [-1,1], we use the center (0) and the maximum (1) to convert the angles into numerical values.
        float numerical_value = (msg->value[i] - robot->front_servos_data[i][5]) / (robot->front_servos_data[i][6] - robot->front_servos_data[i][5]);
        robot->front_prop_array.servos[i].value = numerical_value;
      }
    }
    front_prop_pub->publish(robot->front_prop_array);
  }
}

void RobotNodeHandle::back_topic_callback(smov_states_msgs::msg::StatesServos::SharedPtr msg) {
  if (robot->state == "None") {
    robot->state = msg->state_name.c_str();
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Detecting a new state: %s", robot->state.c_str());
    RCLCPP_INFO(this->get_logger(), "===========================================");
  }

  if (msg->state_name == robot->state) { 
    for (int i = 0; i < SERVO_MAX_SIZE; i++) {
      if (msg->value[i] < (2 * robot->back_servos_data[i][5]) - robot->back_servos_data[i][6]) {
        RCLCPP_ERROR(this->get_logger(), "===========================================");
        RCLCPP_ERROR(this->get_logger(), "Angle out of range (minimum)");
        RCLCPP_ERROR(this->get_logger(), "===========================================");
      } else if (msg->value[i] > robot->back_servos_data[i][6]) {
        RCLCPP_ERROR(this->get_logger(), "===========================================");
        RCLCPP_ERROR(this->get_logger(), "Angle out of range (maximum)");
        RCLCPP_ERROR(this->get_logger(), "===========================================");
      } else {
        float numerical_value = (msg->value[i] - robot->back_servos_data[i][5]) / (robot->back_servos_data[i][6] - robot->back_servos_data[i][5]);
        if (robot->use_single_board) {
          robot->single_back_array.servos[i].value = numerical_value;
        } else {
          robot->back_prop_array.servos[i].value = numerical_value;
        }
      }
    }
  }
  if (robot->use_single_board) {
    front_prop_pub->publish(robot->single_back_array);
  } else {
    back_prop_pub->publish(robot->back_prop_array);
  }
}

void RobotNodeHandle::end_state_callback(smov_states_msgs::msg::EndState::SharedPtr msg) {
  if (msg->state_name == robot->state) {
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "State has shutdown: %s", msg->state_name.c_str());
    RCLCPP_INFO(this->get_logger(), "===========================================");
    robot->state = "None";
  }
}

void RobotNodeHandle::declare_parameters() {
  std::vector<long int> default_value(8, 0); // 8: number of elements in the .yaml array.

  // Declaring all default parameters first.
  for (size_t j = 0; j < robot->servo_name.size(); j++)
    this->declare_parameter(robot->servo_name[j], default_value);

  // Declaring the default values.
  this->declare_parameter("use_single_board", robot->use_single_board);
  this->declare_parameter("upper_leg_length", robot->upper_leg_length);
  this->declare_parameter("lower_leg_length", robot->lower_leg_length);
  this->declare_parameter("hip_body_distance", robot->hip_body_distance);

  // Getting the new parameters from the user.
  robot->use_single_board = this->get_parameter("use_single_board").as_bool();
  robot->upper_leg_length = this->get_parameter("upper_leg_length").as_double();
  robot->lower_leg_length = this->get_parameter("lower_leg_length").as_double();
  robot->hip_body_distance = this->get_parameter("hip_body_distance").as_double();

  // We initialize the arrays with their default values.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    robot->front_servos_data.push_back(this->get_parameter(robot->servo_name[i]).as_integer_array());
    robot->back_servos_data.push_back(this->get_parameter(robot->servo_name[i + SERVO_MAX_SIZE]).as_integer_array());
  }
}

void RobotNodeHandle::set_up_servos() {
  // Initializing with default values.
  std::array<i2c_pwm_board_msgs::msg::Servo, SERVO_MAX_SIZE> empty_front_servos;
  std::array<i2c_pwm_board_msgs::msg::Servo, SERVO_MAX_SIZE> empty_back_servos;

  // Pushing the initial servos to the array.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    robot->front_prop_array.servos.push_back(empty_front_servos[i]);
    robot->back_prop_array.servos.push_back(empty_back_servos[i]);
    if (robot->use_single_board) robot->single_back_array.servos.push_back(empty_front_servos[i]);
  }

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    robot->front_prop_array.servos[i].servo = static_cast<int16_t>(robot->front_servos_data[i][0] + 1); // Port is at position 0.
    robot->back_prop_array.servos[i].servo = static_cast<int16_t>(robot->back_servos_data[i][0] + 1);  // Servo number = Port + 1.

    robot->front_prop_array.servos[i].value = static_cast<float >(robot->front_servos_data[i][4]); // Port is at position 0.
    robot->back_prop_array.servos[i].value = static_cast<float>(robot->back_servos_data[i][4]);  // Servo number = Port + 1.

    if (robot->use_single_board) {
      robot->single_back_array.servos[i].servo = static_cast<int16_t>(robot->back_servos_data[i][0] + 1);
      robot->single_back_array.servos[i].value = static_cast<float>(robot->back_servos_data[i][4]);
    }
  }
}

void RobotNodeHandle::set_up_topics() {
  front_states_sub = this->create_subscription<smov_states_msgs::msg::StatesServos>(
      "front_proportional_servos", 1, std::bind(&RobotNodeHandle::front_topic_callback, this, std::placeholders::_1));

  back_states_sub = this->create_subscription<smov_states_msgs::msg::StatesServos>(
      "back_proportional_servos", 1, std::bind(&RobotNodeHandle::back_topic_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Set up states subscribers.");

  // Setting up the servo config client.
  front_servo_config_client = this->create_client<i2c_pwm_board_msgs::srv::ServosConfig>("front_config_servos");
  if (!robot->use_single_board)
    back_servo_config_client = this->create_client<i2c_pwm_board_msgs::srv::ServosConfig>("back_config_servos");

  front_stop_servos_client = this->create_client<std_srvs::srv::Empty>("front_stop_servos");
  if (!robot->use_single_board) back_stop_servos_client = this->create_client<std_srvs::srv::Empty>("back_stop_servos");

  RCLCPP_INFO(this->get_logger(), "Set up /config_servos_handler publisher.");

  end_state_sub = this->create_subscription<smov_states_msgs::msg::EndState>(
      "end_state", 1, std::bind(&RobotNodeHandle::end_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Set up /end_state subscriber.");

  front_prop_pub = this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("front_servos_proportional", 100);
  if (!robot->use_single_board)
    back_prop_pub = this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("back_servos_proportional", 100);

  RCLCPP_INFO(this->get_logger(), "Set up /servos_proportional_handler publisher.");

  // Setting up the monitor publisher.
  monitor_pub = this->create_publisher<smov_monitor_msgs::msg::DisplayText>("data_display", 1);

  RCLCPP_INFO(this->get_logger(), "Set up /data_display publisher.");
}

void RobotNodeHandle::config_servos() {
  i2c_pwm_board_msgs::msg::ServoConfig front_config[SERVO_MAX_SIZE * 2];
  i2c_pwm_board_msgs::msg::ServoConfig back_config[SERVO_MAX_SIZE];

  auto front_request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();
  auto back_request = std::make_shared<i2c_pwm_board_msgs::srv::ServosConfig::Request>();

  for (int h = 0; h < SERVO_MAX_SIZE; h++) {
    front_config[h].servo = static_cast<int16_t>(robot->front_servos_data[h][0] + 1);
    front_config[h].direction = static_cast<int16_t>(robot->front_servos_data[h][3]);
    front_config[h].center = static_cast<int16_t>(robot->front_servos_data[h][1]);
    front_config[h].range = static_cast<int16_t>(robot->front_servos_data[h][2]);
    front_request->servos.push_back(front_config[h]);

    if (robot->use_single_board) {
      front_config[h + SERVO_MAX_SIZE].servo = static_cast<int16_t>(robot->back_servos_data[h][0] + 1);
      front_config[h + SERVO_MAX_SIZE].direction = static_cast<int16_t>(robot->back_servos_data[h][3]);
      front_config[h + SERVO_MAX_SIZE].center = static_cast<int16_t>(robot->back_servos_data[h][1]);
      front_config[h + SERVO_MAX_SIZE].range = static_cast<int16_t>(robot->back_servos_data[h][2]);
      front_request->servos.push_back(front_config[h + SERVO_MAX_SIZE]);
    } else {
      back_config[h].range = static_cast<int16_t>(robot->back_servos_data[h][2]);
      back_config[h].direction = static_cast<int16_t>(robot->back_servos_data[h][3]);
      back_config[h].center = static_cast<int16_t>(robot->back_servos_data[h][1]);
      back_config[h].servo = static_cast<int16_t>(robot->back_servos_data[h][0] + 1);
      back_request->servos.push_back(back_config[h]);
    }
  }

  while (!front_servo_config_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Front Config Servos service not available, waiting again...");
  }
  if (!robot->use_single_board) {
    while (!back_servo_config_client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Back Config Servos service not available, waiting again...");
    }
  }

  auto f_result = front_servo_config_client->async_send_request(front_request);
  if (!robot->use_single_board) auto b_result = back_servo_config_client->async_send_request(back_request);

  RCLCPP_INFO(this->get_logger(), "Servos have been configured.");
}

void RobotNodeHandle::stop_servos() {
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();

  while (!front_stop_servos_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Front Stop Servos service not available, waiting again...");
  }
  while (!back_stop_servos_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Back Stop Servos service not available, waiting again...");
  }

  auto f_result = front_stop_servos_client->async_send_request(req);
  if (!robot->use_single_board) auto b_result = back_stop_servos_client->async_send_request(req);
}

void RobotNodeHandle::output_values() {
  // Clear the terminal.
  RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");

  for (int b = 0; b < SERVO_MAX_SIZE; b++) {
    RCLCPP_INFO(this->get_logger(), "Front Servo Array Servo %d [value=%f].",
                robot->front_prop_array.servos[b].servo, robot->front_prop_array.servos[b].value);
    if (b == SERVO_MAX_SIZE - 1) {
      RCLCPP_INFO(this->get_logger(), "-------------------------------------------");
      for (int a = 0; a < SERVO_MAX_SIZE; a++) {
        if (robot->use_single_board) {
          RCLCPP_INFO(this->get_logger(), "Back Servo Array Servo %d [value=%f].",
                      robot->single_back_array.servos[a].servo, robot->single_back_array.servos[a].value);
        } else {
          RCLCPP_INFO(this->get_logger(), "Back Servo Array Servo %d [value=%f].",
                      robot->back_prop_array.servos[a].servo, robot->back_prop_array.servos[a].value);
        }
        if (a == SERVO_MAX_SIZE - 1) {
          RCLCPP_INFO(this->get_logger(), "-------------------------------------------");
          RCLCPP_INFO(this->get_logger(), "Current State: %s", robot->state.c_str());
          RCLCPP_INFO(this->get_logger(), "-------------------------------------------");
        }
      }
    }
  }


  // Making sure we are on the desired state.
  robot->up_display.data = std::string("Current state: ") + robot->state;

  // Publishing to the panel.
  monitor_pub->publish(robot->up_display);
}

} // namespace smov
