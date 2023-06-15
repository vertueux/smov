#include <states/smov_states.h>
#include <states/smov_behaviors.h>

#include <ctime>

using namespace std::chrono_literals;

namespace smov {

class StatesNode : public rclcpp::Node {
 public:
  StatesNode()
    : Node("smov_states") {   
      // Servos have been configured using the locking function.
      //front_servos = States::configure_front_servos();
      //back_servos = States::configure_back_servos();

      // Basic configuration on start.
      front_servos = Behaviors::lock_front_phase_one();
      back_servos = Behaviors::lock_back_phase_one();

      // Pushing the servos to the array;
      node->push_all_front_servos_in_array(front_servos);
      node->push_all_back_servos_in_array(back_servos);

      RCLCPP_INFO(this->get_logger(), "Locked the servos on port 0 & 15 on both boards [PHASE 1].");

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

    // We wait for 2 seconds then we ask the user if everything is okay.
    if (time(0) - counter > timeout && !done_phase_two) {
      front_servos = Behaviors::lock_front_phase_two();
      back_servos = Behaviors::lock_back_phase_two();
      push_new_array(front_servos, back_servos);

      RCLCPP_INFO(this->get_logger(), "Locked the servos on port 1 & 14 on both boards [PHASE 2].");
      done_phase_two = true;
    }

    // We wait for 2 seconds then we ask the user if everything is okay.
    if (time(0) - counter > (timeout*2) && !done_phase_three) {
      front_servos = Behaviors::lock_front_phase_three();
      back_servos = Behaviors::lock_back_phase_three();
      push_new_array(front_servos, back_servos);

      RCLCPP_INFO(this->get_logger(), "Locked the servos on port 2 & 13 on both boards [PHASE 3].");
      done_phase_three = true;
    }
  }

  void push_new_array(FrontServos f_servos, BackServos b_servos) {
    node->front_array.servos.clear();
    node->back_array.servos.clear();
    node->push_all_front_servos_in_array(f_servos);
    node->push_all_back_servos_in_array(b_servos);
  }

  // Creating the base node with all the necessary data & publishers.
  States* node = States::Instance();

  time_t counter = time(0);
  time_t timeout = 1; 

  bool done_phase_two = false;
  bool done_phase_three = false;

  // Back & front servos.
  FrontServos front_servos;
  BackServos back_servos;
};      

} // namespace smov

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::StatesNode>());
  rclcpp::shutdown();
  return 0;
}
