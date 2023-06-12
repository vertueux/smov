cmake_minimum_required(VERSION 3.8)
project(configuration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(front_board_msgs REQUIRED)
find_package(back_board_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(servos src/smov_servo_controller.cc)

ament_target_dependencies(servos rclcpp std_msgs front_board_msgs back_board_msgs)

install(TARGETS
  servos 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
