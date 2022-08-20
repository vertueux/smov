#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "i2cpwm_board_msgs/msg/servo_array.hpp"
#include "i2cpwm_board_msgs/msg/servo.hpp"

namespace smov {

struct ServoData {
  int value = 306;  // Center value.
  int center = 306; // Center value.
  int minimum = 83;
  int max = 520;
  int direction = 1;
  int id = 0;
};

class ServoControl : public rclcpp::Node {
 public:
  ServoControl() : Node("servo_control") {
    // Set the servos data.
    for (int i = 0; i < number_of_servos; i++) 
      data.push_back({306, 306, 83, 520, 1, i});

    // Initialize the number of servos that SMOV contains. 
    // They will be as their default values.
    for (int i = 0; i < number_of_servos; i++) {
      i2cpwm_board_msgs::msg::Servo temp_servo;
      temp_servo.servo = i;
      temp_servo.value = 0;
      servo_array.servos.push_back(temp_servo);
    }
  }

 private:
  int number_of_servos = 12;
  std::vector<ServoData> data;
  i2cpwm_board_msgs::msg::ServoArray servo_array;
  i2cpwm_board_msgs::msg::ServoArray servo_array_absolute;
};

}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::ServoControl>());
  rclcpp::shutdown();
}