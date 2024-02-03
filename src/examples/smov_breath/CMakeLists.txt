cmake_minimum_required(VERSION 3.8)
project(smov_breath)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smov_states REQUIRED)
find_package(states_msgs REQUIRED)
find_package(smov_trigonometry REQUIRED)

include_directories(include)

set (dependencies 
  rclcpp 
  smov_states
  states_msgs
  smov_trigonometry
)

# You might want to edit this in case of a library or just an executable. 
add_library(breath_lib SHARED src/breath.cc)
ament_target_dependencies(breath_lib ${dependencies})

add_executable(state src/breath.cc)
target_link_libraries(state breath_lib)
ament_target_dependencies(state ${dependencies})

install(TARGETS state breath_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(breath_lib)
ament_export_dependencies(${dependencies})

ament_package()
