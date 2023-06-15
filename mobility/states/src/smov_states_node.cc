#include <states/smov_states.h>
#include <states/smov_behaviors.h>

#include <ctime>

using namespace std::chrono_literals;

namespace smov {

class StatesNode : public rclcpp::Node {
 public:
  StatesNode()
    : Node("smov_states") {   
      // Back & front servos.
      FrontServos front_servos;
      BackServos back_servos;
      
      // By default, setting up the servos.
      front_servos = States::configure_front_servos();
      back_servos = States::configure_back_servos();

      // Basic configuration on start.
      front_servos = Behaviors::lock_all_front_servos();
      back_servos = Behaviors::lock_all_back_servos();

      // Pushing the servos to the array;
      node->push_all_front_servos_in_array(front_servos);
      node->push_all_back_servos_in_array(back_servos);


      RCLCPP_INFO(this->get_logger(), "Locked the servos on port 1,2,13,14 on both boards.");

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

    // We wait for 5 seconds then we ask the user if everything is okay.
    if (time(0) - counter > timeout && !asked) {
      node->call_for_help();
      asked = true;
    }
  }

  // Creating the base node with all the necessary data & publishers.
  States* node = States::Instance();

  time_t counter = time(0);
  time_t timeout = 5;
  bool asked = false;
};      

} // namespace smov

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::StatesNode>());
  rclcpp::shutdown();
  return 0;
}
