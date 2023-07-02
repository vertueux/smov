#include <ctime>

#include <states/smov_states.h>
#include <states/smov_behaviors.h>

#include "front_board_msgs/srv/servos_config.hpp"
#include "front_board_msgs/msg/servo_config.hpp"
#include "back_board_msgs/srv/servos_config.hpp"
#include "back_board_msgs/msg/servo_config.hpp"

using namespace std::chrono_literals;

namespace smov {

class StatesNode : public rclcpp::Node {
 public:
  StatesNode()
    : Node("smov_states") {   
      std::vector<long int> default_value(7, 0);

      // Declaring all default parameters first.
      for (size_t j = 0; j < servo_name.size(); j++) 
        this->declare_parameter(servo_name[j], default_value);

      // Declaring the angle value for a 100 a.v pulse.
      this->declare_parameter("pulse_for_angle", 0.0);

      // We initialize the arrays with their default values.
      for (int i = 0; i < SERVO_MAX_SIZE; i++) { // 7 is the number of data in a single array (in ~/parameters.yaml).
        States::front_servos_data.push_back(this->get_parameter(servo_name[i]).as_integer_array());
        States::back_servos_data.push_back(this->get_parameter(servo_name[i + SERVO_MAX_SIZE]).as_integer_array());
      }

      // Locking the servos on start.
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempt to lock the servos at their initial value.");

      // Default configuration.
      node->set_up_servos(node->front_abs_servos, node->back_abs_servos);
      Behaviors::set_servos_to_center(node->front_abs_servos, node->back_abs_servos);

      // Pushing the initial servos to the array.
      node->push_in_prop_array(node->front_prop_servos, node->back_prop_servos); 
      node->push_in_abs_array(node->front_abs_servos, node->back_abs_servos);

      // Setting up the proportional publishers.
      front_prop_pub  = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_proportional", 1);
      back_prop_pub = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_proportional", 1);

      // Setting up the absolute publishers.
      front_abs_pub = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_absolute", 1);
      back_abs_pub = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_absolute", 1);

      // Configuring the proportional servos with their defined values.
      config_servos();

      // Calling the basic loop with a timeout of 500ms.
      rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(500ms, std::bind(&StatesNode::call, this));
    }

 private:
  void config_servos() {
    // Setting up the servo config client.
    front_servo_config_pub = this->create_client<front_board_msgs::srv::ServosConfig>("config_servos");
    back_servo_config_pub = this->create_client<back_board_msgs::srv::ServosConfig>("config_servos");

    front_board_msgs::msg::ServoConfig front_config[SERVO_MAX_SIZE];
    back_board_msgs::msg::ServoConfig back_config[SERVO_MAX_SIZE];

    auto front_request = std::make_shared<front_board_msgs::srv::ServosConfig::Request>();
    auto back_request = std::make_shared<back_board_msgs::srv::ServosConfig::Request>();

    for (int h = 0; h < SERVO_MAX_SIZE; h++) {
      front_config[h].servo = int(States::front_servos_data[h][0] + 1);
      back_config[h].servo = int(States::back_servos_data[h][0] + 1);

      front_config[h].center = int(States::front_servos_data[h][1]);
      back_config[h].center = int(States::back_servos_data[h][1]);

      front_config[h].range = int(States::front_servos_data[h][2]);
      back_config[h].range = int(States::back_servos_data[h][2]);

      front_config[h].direction = int(States::front_servos_data[h][3]);
      back_config[h].direction = int(States::back_servos_data[h][3]);

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
    while (!back_servo_config_pub->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    } 
    auto f_result = front_servo_config_pub->async_send_request(front_request);
    auto b_result = back_servo_config_pub->async_send_request(back_request);
  }

  void call() {
    // Publishing the proportional values.
    front_prop_pub->publish(node->front_prop_array);
    back_prop_pub->publish(node->back_prop_array);

    // Publishing the absolute values.
    front_abs_pub->publish(node->front_abs_array);
    back_abs_pub->publish(node->back_abs_array);

    // For now, we are basically nesting loops, cool right...
    for (int i = 0; i < SERVO_MAX_SIZE; i++) { 
      States::front_servos_data[i] = this->get_parameter(servo_name[i]).as_integer_array();
      States::back_servos_data[i] = this->get_parameter(servo_name[i + SERVO_MAX_SIZE]).as_integer_array();
    }

    // Updating the angle for 100 pulse.
    States::pulse_for_angle = this->get_parameter("pulse_for_angle").as_double();
  }

  // Creating the base node with all the necessary data & publishers.
  States* node = States::Instance();

  std::array<std::string, 12> servo_name = {"AVCG", "AVCD", "AVMG", 
                                            "AVMD", "AVJG", "AVJD",
                                            "ARMG", "ARMD", "ARCG",
                                            "ARCD", "ARJG", "ARJD"};
  
  rclcpp::Client<front_board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_pub;
  rclcpp::Client<back_board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_pub;
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_prop_pub;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_prop_pub;
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_abs_pub;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_abs_pub;
};      

} // namespace smov

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::StatesNode>());
  rclcpp::shutdown();
  return 0;
}
