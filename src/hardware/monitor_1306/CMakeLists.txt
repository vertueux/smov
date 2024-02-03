cmake_minimum_required(VERSION 3.8)
project(monitor_1306)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(monitor_msgs REQUIRED)

include_directories(include)

add_executable(node src/monitor_1306.cpp src/monitor_1306_handler.cpp)

ament_target_dependencies(node rclcpp monitor_msgs)

install(TARGETS
        node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_package()
