cmake_minimum_required(VERSION 3.8)
project(smov_states)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(board_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(states_msgs REQUIRED)
find_package(monitor_msgs REQUIRED)

include_directories(include)

set (dependencies 
  rclcpp 
  std_srvs 
  std_msgs
  board_msgs 
  states_msgs
  monitor_msgs
)

add_library(states_lib SHARED src/robot_manager.cc src/robot_node_handler.cc src/robot_states.cc)
ament_target_dependencies(states_lib ${dependencies})

add_executable(manager src/robot_main.cc)
target_link_libraries(manager states_lib)
ament_target_dependencies(manager ${dependencies})

install(TARGETS manager states_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(states_lib)
ament_export_dependencies(${dependencies})

ament_package()
