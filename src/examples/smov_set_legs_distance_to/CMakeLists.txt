cmake_minimum_required(VERSION 3.8)
project(smov_set_legs_distance_to)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smov_states REQUIRED)
find_package(smov_states_msgs REQUIRED)
find_package(smov_trigonometry REQUIRED)

include_directories(include)

set (dependencies 
  rclcpp 
  smov_states
  smov_states_msgs
  smov_trigonometry
)

# You might want to edit this in case of a library or just an executable. 
add_library(set_legs_distance_to_lib SHARED src/set_legs_distance_to.cc)
ament_target_dependencies(set_legs_distance_to_lib ${dependencies})

add_executable(state src/set_legs_distance_to.cc)
target_link_libraries(state set_legs_distance_to_lib)
ament_target_dependencies(state ${dependencies})

install(TARGETS state set_legs_distance_to_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(set_legs_distance_to_lib)
ament_export_dependencies(${dependencies})

ament_package()
