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

add_executable(basic include/states/smov_states.h src/smov_states.cc src/smov_states_node.cc src/smov_behaviors.h src/smov_behaviors.cc)

ament_target_dependencies(basic rclcpp std_msgs front_board_msgs back_board_msgs)

target_include_directories(basic
 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  basic
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
