cmake_minimum_required(VERSION 3.5)
project(frame_visualizer)

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
find_package(px4_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

add_executable(
    frame_visualizer
    src/main.cpp
)

target_link_libraries(
    frame_visualizer
    Eigen3::Eigen
)

ament_target_dependencies(
    frame_visualizer
    rclcpp
    px4_msgs
    nav_msgs
    geometry_msgs
    tf2_ros
)

install(
    TARGETS frame_visualizer
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()