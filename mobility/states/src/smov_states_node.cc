#include <states/smov_states.h>
#include "smov_behaviors.h"

using namespace std::chrono_literals;

namespace smov {

class StatesNode : public rclcpp::Node {
  public:
    StatesNode()
    : Node("smov_states") {   
      // Creating the behaviors manager.
      Behaviors behaviors;

      // Back & front servos.
      FrontServos front_servos;
      BackServos back_servos;
      
      // Setting up the servos.
      node->configure_front_servos(front_servos);
      node->configure_back_servos(back_servos);

      // Pushing the servos to the array;
      node->push_all_front_servos_in_array(front_servos);
      node->push_all_back_servos_in_array(back_servos);

      // Basic configuration on start.
      behaviors.lock_all_front_servos(front_servos);
      behaviors.lock_all_back_servos(back_servos);

      RCLCPP_INFO(this->get_logger(), "Locked the servos on port 1,2,13,14 on both boards.");

      // Setting up the publishers.
      node->front_publisher = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_absolute", 10);
      node->back_publisher = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_absolute", 10);
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
