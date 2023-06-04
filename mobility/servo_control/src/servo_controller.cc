#include <functional>
#include <memory>
#include <termios.h>
#include <unistd.h>

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "front_i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "front_i2c_pwm_board_msgs/msg/servo.hpp"

namespace smov {

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           
  newt = oldt; 
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  

  int c = getchar();  // Read character (non-blocking).

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  
  return c;
}

class ServoControl : public rclcpp::Node {
 public:
  ServoControl() 
   : Node("servo_control") {
    RCLCPP_INFO(this->get_logger(), message);

    front_i2c_pwm_board_msgs::msg::Servo temp_servo;

    // We set all the known servos to their original position.
    for (int i = 1; i < number_of_servos + 1; i++) {
      temp_servo.servo = i;
      temp_servo.value = 0;
      servo_array.servos.push_back(temp_servo);
    }
    
    publisher = this->create_publisher<front_i2c_pwm_board_msgs::msg::ServoArray>("servos_absolute", 1);
    while (rclcpp::ok()) { 
      char terminal_reader = getch();
      switch(terminal_reader) {
        case '1':
          RCLCPP_INFO(this->get_logger(), "Ending program.");
          exit(0);
          break;
        case '2':
          RCLCPP_INFO(this->get_logger(), "Reset all servos to center value.");
          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            servo_array.servos[i].value = center;
          }
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case '3':
          RCLCPP_INFO(this->get_logger(), "Reset all servos to 0.");
          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            servo_array.servos[i].value = 0;
          }
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case '4':
          RCLCPP_INFO(this->get_logger(), "Manually increasing a servo by 1.");
          RCLCPP_INFO(this->get_logger(), "You choose servo number between 1 to 12.");

          std::cin >> rep;
          servo_array.servos[rep - 1].value += 1;
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case '5':
          RCLCPP_INFO(this->get_logger(), "Manually decreasing a servo by 1.");
          RCLCPP_INFO(this->get_logger(), "You choose servo number between 1 to 12.");

          std::cin >> rep;
          servo_array.servos[rep - 1].value -= 1;
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case '6':
          RCLCPP_INFO(this->get_logger(), "Manually increasing a servo by 10.");
          RCLCPP_INFO(this->get_logger(), "You choose servo number between 1 to 12.");

          std::cin >> rep;
          servo_array.servos[rep - 1].value += 10;
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case '7':
          RCLCPP_INFO(this->get_logger(), "Manually decreasing a servo by 10.");
          RCLCPP_INFO(this->get_logger(), "You choose servo number between 1 to 12.");

          std::cin >> rep;
          servo_array.servos[rep - 1].value -= 10;
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case '8':
          RCLCPP_INFO(this->get_logger(), "Reset all servos to maximum value.");
          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            servo_array.servos[i].value = maximum;
          }
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case '9':
          RCLCPP_INFO(this->get_logger(), "Reset all servos to minimum value.");
          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            servo_array.servos[i].value = minimum;
          }
          RCLCPP_INFO(this->get_logger(), message);
          break;
        case 'A':
          RCLCPP_INFO(this->get_logger(), "Setting new center value.");
          RCLCPP_INFO(this->get_logger(), "Choose a new value : ");

          std::cin >> rep;
          center = rep;
          RCLCPP_INFO(this->get_logger(), message);
        case 'B':
          RCLCPP_INFO(this->get_logger(), "Setting new center value.");
          RCLCPP_INFO(this->get_logger(), "Choose a new value : ");

          std::cin >> rep;
          minimum = rep;
          RCLCPP_INFO(this->get_logger(), message);
        case 'C':
          RCLCPP_INFO(this->get_logger(), "Setting new center value.");
          RCLCPP_INFO(this->get_logger(), "Choose a new value : ");

          std::cin >> rep;
          maximum = rep;
          RCLCPP_INFO(this->get_logger(), message);
    }
    publisher->publish(servo_array);
   }
  }

 private:
  int number_of_servos = 16;
  int center = 333;
  int minimum = 83;
  int maximum = 520;

  // Used for answers.
  int rep = 0;

  const char* message = "\n"
  "\nEnter one of the following options:\n"
  "-----------------------------------\n\n"
  "1: Quit.\n"
  "2: Reset all servos to center.\n"
  "3: Reset all servos to 0.\n"
  "4: Increase a servo by 1.\n"
  "5: Decrease a servo by 1.\n"
  "6: Increase a servo by 10.\n"
  "7: Decrease a servo by 10.\n"
  "8: Set all servos to maximum value.\n"
  "9: Set all servos to minimum value.\n"
  "A: Set new center value.\n"
  "B: Set new minimum value.\n"
  "C: Set new maximum value.\n";

  rclcpp::Publisher<front_i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr publisher;

  static front_i2c_pwm_board_msgs::msg::ServoArray servo_array;
};

front_i2c_pwm_board_msgs::msg::ServoArray ServoControl::servo_array;

}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::ServoControl>());
  rclcpp::shutdown();
}
