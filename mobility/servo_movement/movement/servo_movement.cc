#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "i2cpwm_board_msgs/msg/servo_array.hpp"
#include "i2cpwm_board_msgs/msg/servo.hpp"

using std::placeholders::_1;

namespace smov {

constexpr unsigned int str2int(const char* str, int h = 0)
{
  return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

class ServoControl : public rclcpp::Node {
 public:
  ServoControl() 
   : Node("servo_control") {
      RCLCPP_INFO(this->get_logger(), message);

      publisher = this->create_publisher<i2cpwm_board_msgs::msg::ServoArray>("/servos_absolute", 1);
      key_subscription = this->create_subscription<std_msgs::msg::String>(
        "key_event", 10, std::bind(&ServoControl::topic_callback, this, _1));

      for (int i = 0; i < number_of_servos; i++) {
        i2cpwm_board_msgs::msg::Servo temp_servo;

        temp_servo.servo = i;
        temp_servo.value = 0;
        servo_array.servos.push_back(temp_servo);
      }
    }

    void topic_callback(const std_msgs::msg::String& msg) const {
      switch(str2int(msg.data.c_str())) {
        case 'q':
          RCLCPP_INFO(this->get_logger(), "Ending program.");     
          break;
        case 'x':
          RCLCPP_INFO(this->get_logger(), "Reset all servos to center.");
          for (size_t i = 0; i < servo_array.servos.size(); i++) 
            servo_array.servos[i].value = center;
          break;
        case 'r':
          RCLCPP_INFO(this->get_logger(), "Reset all servos to 0.");
          for (size_t i = 0; i < servo_array.servos.size(); i++) 
            servo_array.servos[i].value = 0;
          break;
        case 'i':
          RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");

          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
            servo_array.servos[i].value += 1;
          }
          break;
        case 'd':
          RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");

          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
            servo_array.servos[i].value -= 1;
          }
          break;
        case 'm':
          RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");

          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
            servo_array.servos[i].value = minimum;
          }
          break;
        case 'l':
          RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");

          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
            servo_array.servos[i].value = maximum;
          }
          break;
        case 'c':
          RCLCPP_INFO(this->get_logger(), "Which servo do you want to change the value to? [1, 12]");

          for (size_t i = 0; i < servo_array.servos.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "You choose servo number:'%d' ", static_cast<int>(i));
            servo_array.servos[i].value = center;
          }
          break;
      }
  }

 private:
  int center = 306, minimum = 83, maximum = 520, number_of_servos = 12;

  static i2cpwm_board_msgs::msg::ServoArray servo_array;
  i2cpwm_board_msgs::msg::ServoArray servo_array_absolute;

  rclcpp::Publisher<i2cpwm_board_msgs::msg::ServoArray>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_subscription;
  
  const char* message = "\nServo Control for 12 Servos.\n"
  "\nEnter one of the following options:\n"
  "-----------------------------------\n\n"
  "q: Quit.\n"
  "x: Reset all servos to center.\n"
  "r: Reset all servos to 0.\n"
  "x: Command servo max value.\n"
  "i: Manually decrease a servo command value by 1.\n"
  "d: Manually decrease a servo command value by 1.\n"
  "m: Set a Servo to maximum.\n"
  "n: Set a Servo to minimum.\n"
  "c: Set a Servo to center.\n";
};

i2cpwm_board_msgs::msg::ServoArray ServoControl::servo_array;

}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::ServoControl>());
  rclcpp::shutdown();
}