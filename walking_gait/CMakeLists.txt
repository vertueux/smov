cmake_minimum_required(VERSION 3.8)
project(walking_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libsmov REQUIRED)

include_directories(include)

set (dependencies 
  rclcpp 
  libsmov
)

# You might want to edit this in case of a library or just an executable. 
add_library(walking_gait_lib SHARED src/walking_gait.cc)
ament_target_dependencies(walking_gait_lib ${dependencies})

add_executable(state src/walking_gait.cc)
target_link_libraries(state walking_gait_lib)
ament_target_dependencies(state ${dependencies})

install(TARGETS state walking_gait_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(walking_gait_lib)
ament_export_dependencies(${dependencies})

ament_package()
