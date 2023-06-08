cmake_minimum_required(VERSION 3.8)
project(board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(front_board_msgs REQUIRED)
find_package(back_board_msgs REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(front src/front_i2c_pwm_controller.cc)
target_link_libraries(front i2c)

add_executable(back src/back_i2c_pwm_controller.cc)
target_link_libraries(back i2c)

ament_target_dependencies(front 
  rclcpp 
  std_msgs 
  std_srvs
  front_board_msgs
  xmlrpcpp 
  geometry_msgs
)

ament_target_dependencies(back 
  rclcpp 
  std_msgs 
  std_srvs
  back_board_msgs
  xmlrpcpp 
  geometry_msgs
)

install(TARGETS
  front 
  back
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
