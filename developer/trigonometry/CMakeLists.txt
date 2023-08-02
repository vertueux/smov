cmake_minimum_required(VERSION 3.8)
project(trigonometry)

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

add_library(trigonometry_lib SHARED src/trigonometry.cc)
ament_target_dependencies(trigonometry_lib ${dependencies})

install(TARGETS state trigonometry_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(trigonometry_lib)
ament_export_dependencies(${dependencies})

ament_package()
