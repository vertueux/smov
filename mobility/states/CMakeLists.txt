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

include_directories(include)

set (dependencies 
  rclcpp 
  std_msgs 
  front_board_msgs 
  back_board_msgs
)

add_library(states_lib SHARED src/smov_states.cc src/smov_behaviors.cc)
ament_target_dependencies(states_lib ${dependencies})

add_executable(basic src/smov_states_node.cc)
target_link_libraries(basic states_lib)
ament_target_dependencies(basic ${dependencies})

install(TARGETS basic states_lib
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
