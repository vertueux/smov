#include <ctime>
#include <iostream>
#include <memory>

#include <states/robot_node_handler.h>

using namespace std::chrono_literals;

namespace smov {

RobotNodeHandle::RobotNodeHandle()
 : Node("smov_states") {   

  // Declaring the different parameters.
  declare_parameters();

  // Locking the servos on start.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempt to lock the servos at their initial value.");

  // Initializing with default values.
  std::array<front_board_msgs::msg::Servo, SERVO_MAX_SIZE> empty_front_servos;
  std::array<back_board_msgs::msg::Servo, SERVO_MAX_SIZE> empty_back_servos;

  // Pushing the initial servos to the array.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    robot->front_prop_array.servos.push_back(empty_front_servos[i]);
    robot->back_prop_array.servos.push_back(empty_back_servos[i]);
    robot->front_abs_array.servos.push_back(empty_front_servos[i]);
    robot->back_abs_array.servos.push_back(empty_back_servos[i]);
  }

  // Default configuration.
  robot->set_up_servos();

  // Setting up the publishers.
  set_up_topics();

  // Configuring the proportional servos with their defined values.
  config_servos();

  // Calling the loops with some timeouts.
  timer = this->create_wall_timer(10ms, std::bind(&RobotNodeHandle::callback, this));
  late_timer = this->create_wall_timer(4s, std::bind(&RobotNodeHandle::late_callback, this));
}

void RobotNodeHandle::front_topic_callback(states_msgs::msg::StatesServos::SharedPtr msg) {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) 
    robot->front_prop_array.servos[i].value = msg->value[i];
}

void RobotNodeHandle::back_topic_callback(states_msgs::msg::StatesServos::SharedPtr msg) {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) 
    robot->back_prop_array.servos[i].value = msg->value[i];
}

void RobotNodeHandle::state_topic_callback(std_msgs::msg::String::SharedPtr msg) {
  robot->state = msg->data.c_str();
}

void RobotNodeHandle::declare_parameters() {
  std::vector<long int> default_value(5, 0);

  // Declaring all default parameters first.
  for (size_t j = 0; j < robot->servo_name.size(); j++) 
    this->declare_parameter(robot->servo_name[j], default_value);

  // We initialize the arrays with their default values.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) { // 5 is the number of data in a single array (in ~/parameters.yaml).
    robot->front_servos_data.push_back(this->get_parameter(robot->servo_name[i]).as_integer_array());
    robot->back_servos_data.push_back(this->get_parameter(robot->servo_name[i + SERVO_MAX_SIZE]).as_integer_array());
  }
}

void RobotNodeHandle::set_up_topics() {
  front_states_sub = this->create_subscription<states_msgs::msg::StatesServos>(
  "front_proportional_servos", 1, std::bind(&RobotNodeHandle::front_topic_callback, this, std::placeholders::_1));

  back_states_sub = this->create_subscription<states_msgs::msg::StatesServos>(
  "back_proportional_servos", 1, std::bind(&RobotNodeHandle::back_topic_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Set up states subscribers.");

  // Setting up the servo config client.
  front_servo_config_pub = this->create_client<front_board_msgs::srv::ServosConfig>("config_servos");
  //back_servo_config_pub = this->create_client<back_board_msgs::srv::ServosConfig>("config_servos");

  RCLCPP_INFO(this->get_logger(), "Set up /config_servos publisher.");

  last_current_state_pub  = this->create_subscription<std_msgs::msg::String>(
  "last_current_state", 1, std::bind(&RobotNodeHandle::state_topic_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Set up /last_current_state publisher.");

  front_prop_pub  = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_proportional", 1);
  //back_prop_pub = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_proportional", 1);

  RCLCPP_INFO(this->get_logger(), "Set up /servos_proportional publisher.");

  // Setting up the absolute publishers.
  front_abs_pub = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_absolute", 1);
  //back_abs_pub = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_absolute", 1);

  RCLCPP_INFO(this->get_logger(), "Set up /servos_absolute publisher.");
}

void RobotNodeHandle::config_servos() {
  front_board_msgs::msg::ServoConfig front_config[SERVO_MAX_SIZE];
  back_board_msgs::msg::ServoConfig back_config[SERVO_MAX_SIZE];

  auto front_request = std::make_shared<front_board_msgs::srv::ServosConfig::Request>();
  auto back_request = std::make_shared<back_board_msgs::srv::ServosConfig::Request>();

  for (int h = 0; h < SERVO_MAX_SIZE; h++) {
    front_config[h].servo = int(robot->front_servos_data[h][0] + 1);
    back_config[h].servo = int(robot->back_servos_data[h][0] + 1);

    front_config[h].center = int(robot->front_servos_data[h][1]);
    back_config[h].center = int(robot->back_servos_data[h][1]);

    front_config[h].range = int(robot->front_servos_data[h][2]);
    back_config[h].range = int(robot->back_servos_data[h][2]);

    front_config[h].direction = int(robot->front_servos_data[h][3]);
    back_config[h].direction = int(robot->back_servos_data[h][3]);

    front_request->servos.push_back(front_config[h]); 
    //back_request->servos.push_back(back_config[h]); 
  }

  while (!front_servo_config_pub->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  } 
  /*while (!back_servo_config_pub->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }*/
  auto f_result = front_servo_config_pub->async_send_request(front_request);
  //auto b_result = back_servo_config_pub->async_send_request(back_request);

  RCLCPP_INFO(this->get_logger(), "Servos have been configured.");
}

void RobotNodeHandle::callback() {
  // Publishing the proportional values.
  front_prop_pub->publish(robot->front_prop_array);
  //back_prop_pub->publish(robot->back_prop_array);

  // Publishing the absolute values.
  // FOR NOW WE DON'T PUBLISH ANYTHING AS THIS CAUSE ISSUES.
  // front_abs_pub->publish(robot->front_abs_array);
  // back_abs_pub->publish(robot->back_abs_array);
}

void RobotNodeHandle::late_callback() {
  for (int b = 0; b < SERVO_MAX_SIZE; b++) { 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Front Servo Array Servo %d [value=%f].", 
    robot->front_prop_array.servos[b].servo, robot->front_prop_array.servos[b].value);
    if (b == SERVO_MAX_SIZE - 1) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "-------------------------------------------");
      for (int a = 0; a < SERVO_MAX_SIZE; a++) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Back Servo Array Servo %d [value=%f].", 
        robot->front_prop_array.servos[a].servo, robot->front_prop_array.servos[a].value);
        if (a == SERVO_MAX_SIZE - 1) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
          "-------------------------------------------");
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Last State: %s", robot->state.c_str());
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
          "-------------------------------------------");
        }
      }
    }
  }

  // For now, we are basically nesting loops, cool right...
  for (int i = 0; i < SERVO_MAX_SIZE; i++) { 
    robot->front_servos_data[i] = this->get_parameter(robot->servo_name[i]).as_integer_array();
    robot->back_servos_data[i] = this->get_parameter(robot->servo_name[i + SERVO_MAX_SIZE]).as_integer_array();
  }
}

} // namespace smov
