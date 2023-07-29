#include <calibration/robot_calibration.h>

namespace smov {
class ServoControl : public rclcpp::Node {
 public:
  ServoControl() 
   : Node("servo_control") {
    RCLCPP_INFO(this->get_logger(), Calibration::message);
    set_up_abs_servos();

    while (rclcpp::ok()) { 
      char terminal_reader = Calibration::get_char();
      switch(terminal_reader) {
        case '0': Calibration::exit_program();                                                                    break;
        case '1': Calibration::switch_board();                                                                    break;
        case '2': Calibration::reset_all_servos_to(Calibration::center, "Reset all servos to center value.");   break;
        case '3': Calibration::reset_all_servos_to(Calibration::maximum, "Reset all servos to maximum value."); break;
        case '4': Calibration::reset_all_servos_to(Calibration::minimum, "Reset all servos to minimum value."); break;
        case '5': Calibration::reset_all_servos_to(0, "Reset all servos to 0.");                                  break;
        case '6': Calibration::increase_or_decrease_by(2, true, "Manually increasing a servo by 2.");             break;
        case '7': Calibration::increase_or_decrease_by(2, false, "Manually decreasing a servo by 2.");            break;
        case '8': Calibration::increase_or_decrease_by(10, true, "Manually increasing a servo by 10.");           break;
        case '9': Calibration::increase_or_decrease_by(10, false, "Manually decreasing a servo by 10.");          break;
        case 'A': Calibration::center = Calibration::set_new_value("Setting new center value.");                break;
        case 'B': Calibration::minimum = Calibration::set_new_value("Setting new minimum value.");              break;
        case 'C': Calibration::maximum = Calibration::set_new_value("Setting new maximum value.");              break;
      }

      if (Calibration::active_board == 1)
        front_abs_pub->publish(Calibration::front_servo_array);
      else  
        back_abs_pub->publish(Calibration::back_servo_array);
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

      Calibration::front_servo_array.servos.push_back(front_temp_servo);
      Calibration::back_servo_array.servos.push_back(back_temp_servo);
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
