#include <functional>
#include <memory>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>

#include "i2cpwm_board_msgs/msg/servo_array.hpp"
#include "i2cpwm_board_msgs/msg/servo.hpp"

#define CENTER 306
#define MINIMUM 83
#define MAXIMUM 520

namespace smov {

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt; 
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

class ServoControl : public rclcpp::Node {
 public:
  static int number_of_servos;
  static i2cpwm_board_msgs::msg::ServoArray servo_array;

  ServoControl() 
   : Node("servo_control") {
    publisher = this->create_publisher<i2cpwm_board_msgs::msg::ServoArray>("/servos_absolute", 1);

    char terminal_reader = getch();

    switch(terminal_reader) {
      case 'q':
        RCLCPP_INFO(this->get_logger(), "Ending program.");
        break;
      case 'x':
        RCLCPP_INFO(this->get_logger(), "Reset all servos to center.");
        for (size_t i = 0; i < servo_array.servos.size(); i++) {
          servo_array.servos[i].value = CENTER;
        }
        break;
      case 'r':
        RCLCPP_INFO(this->get_logger(), "Reset all servos to 0.");
        for (size_t i = 0; i < servo_array.servos.size(); i++) {
          servo_array.servos[i].value = 0;
        }
        break;
      case 'i':
        RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");
        for (size_t i = 0; i < servo_array.servos.size() && terminal_reader != static_cast<int>(i); i++) {
          RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
          servo_array.servos[i].value += 1;
        }
        break;
      case 'd':
        RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");
        for (size_t i = 0; i < servo_array.servos.size() && terminal_reader != static_cast<int>(i); i++) {
          RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
          servo_array.servos[i].value -= 1;
        }
        break;
      case 'm':
        RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");
        for (size_t i = 0; i < servo_array.servos.size() && terminal_reader != static_cast<int>(i); i++) {
          RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
          servo_array.servos[i].value = MINIMUM;
        }
        break;
      case 'l':
        RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");
        for (size_t i = 0; i < servo_array.servos.size() && terminal_reader != static_cast<int>(i); i++) {
          RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
          servo_array.servos[i].value = MAXIMUM;
        }
        break;
      case 'c':
        RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");
        for (size_t i = 0; i < servo_array.servos.size() && terminal_reader != static_cast<int>(i); i++) {
          RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
          servo_array.servos[i].value = CENTER;
        }
        break;
    }
  }

 private:
  i2cpwm_board_msgs::msg::ServoArray servo_array_absolute;
  rclcpp::Publisher<i2cpwm_board_msgs::msg::ServoArray>::SharedPtr publisher;
};

int ServoControl::number_of_servos = 12;
i2cpwm_board_msgs::msg::ServoArray ServoControl::servo_array;

}

int main(int argc, char* argv[]) {
  const char* message = "\nServo Control for 12 Servos.\n \nEnter one of the following options:\n-----------------------------------\n \nq: Quit.\nx: Reset all servos to center.\nr: Reset all servos to 0.\nx: Command servo max value.\ni: Manually decrease a servo command value by 1.\nd: Manually decrease a servo command value by 1.\nm: Set a Servo to maximum.\nn: Set a Servo to minimum.\nc: Set a Servo to center.\n";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);

  rclcpp::init(argc, argv);

  for (int i = 0; i < smov::ServoControl::number_of_servos; i++) {
    i2cpwm_board_msgs::msg::Servo temp_servo;

    temp_servo.servo = i;
    temp_servo.value = 0;
    smov::ServoControl::servo_array.servos.push_back(temp_servo);
  }
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(std::make_shared<smov::ServoControl>());
  }
    
  rclcpp::shutdown();
}