cmake_minimum_required(VERSION 3.8)
project(primary_motions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(front_i2c_pwm_board_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wake_up src/robot_wake_up.cc)
add_executable(go_sleep src/robot_go_down.cc)

ament_target_dependencies(wake_up 
  rclcpp 
  std_msgs
  front_i2c_pwm_board_msgs
)

ament_target_dependencies(go_sleep 
  rclcpp 
  std_msgs
  front_i2c_pwm_board_msgs
)

install(TARGETS
  wake_up 
  go_sleep
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
