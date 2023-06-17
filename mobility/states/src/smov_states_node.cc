#include <states/smov_states.h>
#include <states/smov_behaviors.h>

#include <ctime>

using namespace std::chrono_literals;

namespace smov {

class StatesNode : public rclcpp::Node {
 public:
  StatesNode()
    : Node("smov_states") {   
      // Pushing the initial servos to the array;
      node->push_all_servos_in_array(node->front_servos, node->back_servos);


      // Locking the servos on start.
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempt to lock the servos at their initial value.");
      Behaviors::lock_servos(node->front_servos, node->back_servos);

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
  }

  // Creating the base node with all the necessary data & publishers.
  States* node = States::Instance();
};      

} // namespace smov

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::StatesNode>());
  rclcpp::shutdown();
  return 0;
}
