cmake_minimum_required(VERSION 3.8)
project(monitor_1306)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(monitor_msgs REQUIRED)

include_directories(include)

set (dependencies
        rclcpp
        monitor_msgs

)

add_library(monitor_1306_lib SHARED src/monitor_1306.cpp src/monitor_1306_handler.cpp)
ament_target_dependencies(monitor_1306_lib ${dependencies})

add_executable(state src/monitor_1306.cpp)
target_link_libraries(state monitor_1306_lib)
ament_target_dependencies(state ${dependencies})

install(TARGETS state monitor_1306_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(monitor_1306_lib)
ament_export_dependencies(${dependencies})

ament_package()
