cmake_minimum_required(VERSION 3.8)
project(keyboard_axes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(states REQUIRED)
find_package(states_msgs REQUIRED)

include_directories(include)

set (dependencies 
  rclcpp 
  states
  states_msgs
)

add_library(keyboard_axes_lib SHARED src/robot_keyboard_axes.cc)
ament_target_dependencies(keyboard_axes_lib ${dependencies})

add_executable(state src/robot_main.cc)
target_link_libraries(state keyboard_axes_lib)
ament_target_dependencies(state ${dependencies})

install(TARGETS state keyboard_axes_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(keyboard_axes_lib)
ament_export_dependencies(${dependencies})

ament_package()
