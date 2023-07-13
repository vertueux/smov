#include <configuration/smov_configuration.h>

namespace smov {
class ServoControl : public rclcpp::Node {
 public:
  ServoControl() 
   : Node("servo_control") {
    RCLCPP_INFO(this->get_logger(), Configuration::message);
    set_up_abs_servos();

    while (rclcpp::ok()) { 
      char terminal_reader = Configuration::get_char();
      switch(terminal_reader) {
        case '0': Configuration::exit_program();                                                                break;
        case '1': Configuration::switch_board();                                                                break;
        case '2': Configuration::reset_all_servos_to(Configuration::center, "Reset all servos to center value.");   break;
        case '3': Configuration::reset_all_servos_to(Configuration::maximum, "Reset all servos to maximum value."); break;
        case '4': Configuration::reset_all_servos_to(Configuration::minimum, "Reset all servos to minimum value."); break;
        case '5': Configuration::reset_all_servos_to(0, "Reset all servos to 0.");                                  break;
        case '6': Configuration::increase_or_decrease_by(2, true, "Manually increasing a servo by 2.");         break;
        case '7': Configuration::increase_or_decrease_by(2, false, "Manually decreasing a servo by 2.");        break;
        case '8': Configuration::increase_or_decrease_by(10, true, "Manually increasing a servo by 10.");       break;
        case '9': Configuration::increase_or_decrease_by(10, false, "Manually decreasing a servo by 10.");      break;
        case 'A': Configuration::center = Configuration::set_new_value("Setting new center value.");            break;
        case 'B': Configuration::minimum = Configuration::set_new_value("Setting new minimum value.");          break;
        case 'C': Configuration::maximum = Configuration::set_new_value("Setting new maximum value.");          break;
      }

      if (Configuration::active_board == 1)
        front_abs_pub->publish(Configuration::front_servo_array);
      else  
        back_abs_pub->publish(Configuration::back_servo_array);
    }
  }

  void set_up_abs_servos() {
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

    front_abs_pub = this->create_publisher<front_board_msgs::msg::ServoArray>("servos_absolute", 1);
    back_abs_pub = this->create_publisher<back_board_msgs::msg::ServoArray>("servos_absolute", 1);
  }

 private:
  int number_of_servos = 16;

  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_abs_pub;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_abs_pub;
};

} // namespace smov

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::ServoControl>());
  rclcpp::shutdown();
}
