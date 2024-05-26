cmake_minimum_required(VERSION 3.8)
project(smov_base)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

set(dependencies
        rclcpp
)

add_library(smov_base_lib SHARED src/base.cc)
ament_target_dependencies(smov_base_lib ${dependencies})

install(TARGETS smov_base_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(smov_base_lib)
ament_export_dependencies(${dependencies})

ament_package()
