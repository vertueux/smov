#include <configuration/smov_configuration.h>

namespace smov {
class ServoControl : public rclcpp::Node {
 public:
  ServoControl() 
   : Node("servo_control") {
    RCLCPP_INFO(this->get_logger(), Configuration::message);
    set_up_servos();

    while (rclcpp::ok()) { 
      char terminal_reader = Configuration::get_char();
      switch(terminal_reader) {
        case '0': Configuration::exit_program();           break;
        case '1': Configuration::switch_board();           break;
        case '2': Configuration::reset_servos_to_center(); break;
        case '3': Configuration::reset_servos_to_zero();   break;
        case '4': Configuration::increase_servo_by_one();  break;
        case '5': Configuration::decrease_servo_by_one();  break;
        case '6': Configuration::increase_servo_by_ten();  break;
        case '7': Configuration::decrease_servo_by_ten();  break;
        case '8': Configuration::reset_to_maximum_value(); break;
        case '9': Configuration::reset_to_minimum_value(); break;
        case 'A': Configuration::set_new_center_value();   break;
        case 'B': Configuration::set_new_minimum_value();  break;
        case 'C': Configuration::set_new_maximum_value();  break;
      }

      if (Configuration::active_board == 1)
        front_publisher->publish(Configuration::front_servo_array);
      else  
        back_publisher->publish(Configuration::back_servo_array);
    }
  }

  void set_up_servos() {
    front_board_msgs::msg::Servo front_temp_servo;
    back_board_msgs::msg::Servo back_temp_servo;

    // We set all the known servos to their original position.
    for (int i = 1; i < number_of_servos; i++) {
      // Front board.
      front_temp_servo.servo = i;
      front_temp_servo.value = 0;

      // Back board.
      back_temp_servo.servo = i;
      back_temp_servo.value = 0;

      Configuration::front_servo_array.servos.push_back(front_temp_servo);
      Configuration::back_servo_array.servos.push_back(back_temp_servo);
    }

    front_publisher = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_absolute", 1);
    back_publisher = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_absolute", 1);
  }

 private:
  int number_of_servos = 16;

  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_publisher;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_publisher;
};

} // namespace smov

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::ServoControl>());
  rclcpp::shutdown();
}
