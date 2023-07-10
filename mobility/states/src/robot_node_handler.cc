#include <ctime>
#include <iostream>

#include <states/robot_node_handler.h>

using namespace std::chrono_literals;

namespace smov {

RobotNodeHandle::RobotNodeHandle()
 : Node("smov_states") {   

  // Declaring the different parameters.
  declare_parameters();

  // Locking the servos on start.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempt to lock the servos at their initial value.");

  // Default configuration.
  node->set_up_servos();

  // Configuration on start.
  node->on_start();

  // Pushing the initial servos to the array.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    node->front_prop_array.servos.push_back(node->front_prop_servos[i]);
    node->back_prop_array.servos.push_back(node->back_prop_servos[i]);
  }
  
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    node->front_abs_array.servos.push_back(node->front_abs_servos[i]);
    node->back_abs_array.servos.push_back(node->back_abs_servos[i]);
  }

  // Setting up the publishers.
  set_up_publishers();

  // Configuring the proportional servos with their defined values.
  config_servos();

  // Calling the basic loop with a timeout of 500ms.
  timer = this->create_wall_timer(500ms, std::bind(&RobotNodeHandle::call, this));
}

void RobotNodeHandle::declare_parameters() {
  std::vector<long int> default_value(7, 0);

  // Declaring all default parameters first.
  for (size_t j = 0; j < node->servo_name.size(); j++) 
    this->declare_parameter(node->servo_name[j], default_value);

  // Declaring the angle value for a 100 a.v pulse.
  this->declare_parameter("pulse_for_angle", 0.0);

  // We initialize the arrays with their default values.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) { // 7 is the number of data in a single array (in ~/parameters.yaml).
    node->front_servos_data.push_back(this->get_parameter(node->servo_name[i]).as_integer_array());
    node->back_servos_data.push_back(this->get_parameter(node->servo_name[i + SERVO_MAX_SIZE]).as_integer_array());
  }
}

void RobotNodeHandle::set_up_publishers() {
  // Setting up the servo config client.
  front_servo_config_pub = this->create_client<front_board_msgs::srv::ServosConfig>("config_servos");
  //back_servo_config_pub = this->create_client<back_board_msgs::srv::ServosConfig>("config_servos");

  RCLCPP_INFO(this->get_logger(), "Set up /config_servos publisher.");

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
    front_config[h].servo = int(node->front_servos_data[h][0] + 1);
    back_config[h].servo = int(node->back_servos_data[h][0] + 1);

    front_config[h].center = int(node->front_servos_data[h][1]);
    back_config[h].center = int(node->back_servos_data[h][1]);

    front_config[h].range = int(node->front_servos_data[h][2]);
    back_config[h].range = int(node->back_servos_data[h][2]);

    front_config[h].direction = int(node->front_servos_data[h][3]);
    back_config[h].direction = int(node->back_servos_data[h][3]);

    front_request->servos.push_back(front_config[h]); 
    back_request->servos.push_back(back_config[h]); 
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

void RobotNodeHandle::call() {
  // Configuration on the loop.
  node->on_loop();

  for (int b = 0; b < SERVO_MAX_SIZE; b++) {
    std::cout << node->front_prop_array.servos[b].value << std::endl;
  }

  // Constantly updating the values.
  node->update_servos_arrays();

  // Publishing the proportional values.
  front_prop_pub->publish(node->front_prop_array);
  //back_prop_pub->publish(node->back_prop_array);

  // Publishing the absolute values.
  // FOR NOW WE DON'T PUBLISH ANYTHING AS THIS CAUSE ISSUES.
  // front_abs_pub->publish(node->front_abs_array);
  // back_abs_pub->publish(node->back_abs_array);

  // For now, we are basically nesting loops, cool right...
  for (int i = 0; i < SERVO_MAX_SIZE; i++) { 
    node->front_servos_data[i] = this->get_parameter(node->servo_name[i]).as_integer_array();
    node->back_servos_data[i] = this->get_parameter(node->servo_name[i + SERVO_MAX_SIZE]).as_integer_array();
  }

  // Updating the pulse for angle.
  node->pulse_for_angle = this->get_parameter("pulse_for_angle").as_double();
}

} // namespace smov
