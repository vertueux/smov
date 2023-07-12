cmake_minimum_required(VERSION 3.8)
project(manual_wake_up)

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

add_library(manual_wake_up_lib SHARED src/robot_manual_wake_up.cc)
ament_target_dependencies(manual_wake_up_lib ${dependencies})

add_executable(state src/robot_main.cc)
target_link_libraries(state manual_wake_up_lib)
ament_target_dependencies(state ${dependencies})

install(TARGETS state manual_wake_up_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(manual_wake_up_lib)
ament_export_dependencies(${dependencies})

ament_package()
