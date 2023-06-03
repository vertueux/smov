#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "front_i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "front_i2c_pwm_board_msgs/msg/servo.hpp"

namespace smov {

class WakeUp : public rclcpp::Node {
 public:
  WakeUp() 
   : Node("smov_wake_up") {
    configure_servos(servo_array);
    
    publisher = this->create_publisher<front_i2c_pwm_board_msgs::msg::ServoArray>("servos_absolute", 1);
    publisher->publish(servo_array);

    exit(0);
  }
  
  // Theses commands are specific to the hardware chosen.
  void configure_servos(front_i2c_pwm_board_msgs::msg::ServoArray array) {
    // Setting the front left servo.
    front_i2c_pwm_board_msgs::msg::Servo front_left_servo;
    front_left_servo.servo = 1;
    front_left_servo.value = -1;
    array.servos.push_back(front_left_servo);

    // Setting the front right servo.
    front_i2c_pwm_board_msgs::msg::Servo front_right_servo;
    front_right_servo.servo = 16;
    front_right_servo.value = -1;
    array.servos.push_back(front_right_servo);

    // Setting the back left servo.
    front_i2c_pwm_board_msgs::msg::Servo back_left_servo;
    back_left_servo.servo = 2;
    back_left_servo.value = -1;
    array.servos.push_back(back_left_servo);

    // Setting the back right servo.
    front_i2c_pwm_board_msgs::msg::Servo back_right_servo;
    back_right_servo.servo = 15;
    back_right_servo.value = -1;
    array.servos.push_back(back_right_servo);
  }

 private:
  rclcpp::Publisher<front_i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr publisher;
  static front_i2c_pwm_board_msgs::msg::ServoArray servo_array;
};

front_i2c_pwm_board_msgs::msg::ServoArray WakeUp::servo_array;

}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::WakeUp>());
  rclcpp::shutdown();
}
