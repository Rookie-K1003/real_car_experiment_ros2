cmake_minimum_required(VERSION 3.5)
project(adas)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -g ")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

message("CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(location_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
	include
  ${EIGEN3_INCLUDE_DIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(map_tracking
    src/map_tracking.cpp
    src/plan.cpp
    src/control.cpp
    src/adas_utils.cpp
)

ament_target_dependencies(map_tracking
    rclcpp
    tf2_ros
    std_msgs
    nav_msgs
    geometry_msgs
    location_msgs
    control_msgs
)

add_executable(map_generator
    src/map_generator.cpp
    src/rtk_map.cpp
    src/adas_utils.cpp
)

ament_target_dependencies(map_generator
    rclcpp
    tf2_ros
    std_msgs
    nav_msgs
    geometry_msgs
    location_msgs
    control_msgs
)

add_executable(debug
    src/debug.cpp
)

ament_target_dependencies(debug
    rclcpp
    location_msgs
)

# 用于模拟期望控制信号发布，用于CAN通信测试
add_executable(control_test
    src/control_test.cpp
)

ament_target_dependencies(control_test
    rclcpp
    control_msgs
    geometry_msgs # 20250416
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  map_tracking
  map_generator
  debug
  control_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()