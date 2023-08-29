cmake_minimum_required(VERSION 3.8)
project(smov_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(board_msgs REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(controller src/dual_board_controller.cc)
target_link_libraries(controller i2c)

ament_target_dependencies(controller rclcpp std_msgs std_srvs board_msgs xmlrpcpp geometry_msgs)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
