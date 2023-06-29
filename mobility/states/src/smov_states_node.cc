#include <states/smov_states.h>
#include <states/smov_behaviors.h>

#include <ctime>

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

      // We initialize the arrays with their default values.
      for (int i = 0; i < SERVO_MAX_SIZE; i++) { // 7 is the number of data in a single array (in ~/parameters.yaml).
        States::front_servos_data.push_back(this->get_parameter(servo_name[i]).as_integer_array());
        States::back_servos_data.push_back(this->get_parameter(servo_name[i + SERVO_MAX_SIZE]).as_integer_array());
      }

      // Locking the servos on start.
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempt to lock the servos at their initial value.");

      // Default configuration.
      node->set_up_servos(node->front_servos, node->back_servos);
      Behaviors::set_servos_to_center(node->front_servos, node->back_servos);

      // Pushing the initial servos to the array;
      node->push_all_servos_in_array(node->front_servos, node->back_servos);

      // Setting up the publishers.
      node->front_publisher = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_absolute", 1);
      node->back_publisher = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_absolute", 1);

      node->timer = this->create_wall_timer(500ms, std::bind(&StatesNode::call, this));
    }

 private:
  void call() {
    // Publishing all necessary data to servos.
    node->front_publisher->publish(node->front_array);
    node->back_publisher->publish(node->back_array);

    // For now, we are basically nesting loops, cool right...
    for (int i = 0; i < SERVO_MAX_SIZE; i++) { 
      States::front_servos_data[i] = this->get_parameter(servo_name[i]).as_integer_array();
      States::back_servos_data[i] = this->get_parameter(servo_name[i + SERVO_MAX_SIZE]).as_integer_array();
    }
  }

  // Creating the base node with all the necessary data & publishers.
  States* node = States::Instance();

  std::array<std::string, 12> servo_name = {"AVCG", "AVCD", "AVMG", 
                                            "AVMD", "AVJG", "AVJD",
                                            "ARMG", "ARMD", "ARCG",
                                            "ARCD", "ARJG", "ARJD"};
};      

} // namespace smov

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::StatesNode>());
  rclcpp::shutdown();
  return 0;
}
