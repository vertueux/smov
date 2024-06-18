cmake_minimum_required(VERSION 3.8)
project(libsmov)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smov_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  std_srvs
  std_msgs
  smov_msgs
)

add_library(smov_lib SHARED src/base.cc src/executable.cc src/library.cc src/mathematics.cc src/trigonometry.cc)
ament_target_dependencies(smov_lib ${dependencies})

install(TARGETS smov_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(smov_lib)
ament_export_dependencies(${dependencies})

ament_package()
