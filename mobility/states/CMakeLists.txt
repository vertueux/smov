cmake_minimum_required(VERSION 3.8)
project(states)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(front_board_msgs REQUIRED)
find_package(back_board_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(wake_up src/smov_wake_up.cc)
add_executable(go_sleep src/smov_go_down.cc)
add_executable(lock src/smov_lock.cc)

ament_target_dependencies(basic rclcpp std_msgs front_board_msgs back_board_msgs)

install(TARGETS
  basic
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
