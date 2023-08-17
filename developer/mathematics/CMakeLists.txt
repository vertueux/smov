cmake_minimum_required(VERSION 3.8)
project(smov_mathematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smov_states REQUIRED)
find_package(states_msgs REQUIRED)

include_directories(include)

set (dependencies 
  rclcpp 
  smov_states
  states_msgs
)

add_library(mathematics_lib SHARED src/mathematics.cc)
ament_target_dependencies(mathematics_lib ${dependencies})

install(TARGETS mathematics_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(mathematics_lib)
ament_export_dependencies(${dependencies})

ament_package()
