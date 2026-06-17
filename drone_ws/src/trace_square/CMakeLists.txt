cmake_minimum_required(VERSION 3.5)
project(trace_square)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Wno-unused-parameter
    )
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

add_executable(
    trace_square
    src/main.cpp
)

target_link_libraries(
    trace_square
    Eigen3::Eigen
)

ament_target_dependencies(
    trace_square
    rclcpp
    px4_ros2_cpp
)

install(
    TARGETS trace_square
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()