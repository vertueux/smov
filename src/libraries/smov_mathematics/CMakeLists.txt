cmake_minimum_required(VERSION 3.8)
project(smov_mathematics)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

set(dependencies
        rclcpp
)

add_library(smov_mathematics_lib SHARED src/mathematics.cc)
ament_target_dependencies(smov_mathematics_lib ${dependencies})

install(TARGETS smov_mathematics_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(smov_mathematics_lib)
ament_export_dependencies(${dependencies})

ament_package()
