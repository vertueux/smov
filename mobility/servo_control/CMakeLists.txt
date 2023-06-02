cmake_minimum_required(VERSION 3.8)
project(servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(front_i2c_pwm_board_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(servo_control_executable src/servo_controller.cc)

ament_target_dependencies(servo_control_executable 
  rclcpp 
  std_msgs
  front_i2c_pwm_board_msgs
)

install(TARGETS
  servo_control_executable 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
