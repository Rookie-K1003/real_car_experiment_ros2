cmake_minimum_required(VERSION 3.5)
project(can_decode)

# add_compile_options(-std=c++14)
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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(location_msgs REQUIRED)
#find_package(message_generation REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIB_DIRS})

# CAN解析节点
add_executable(can_decode
    src/ins_decode_V1.cpp)

# 可视化节点
add_executable(ins_visualizer
    src/ins_visualizer.cpp)

# 添加 lla2map_converter 的源文件
add_library(lla2map_converter
    src/lla2map_converter.cpp
)

# 链接 lla2map_converter 库
target_include_directories(lla2map_converter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(can_decode
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    location_msgs
)

ament_target_dependencies(ins_visualizer
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    location_msgs
)

target_link_libraries(can_decode
  ${catkin_LIBRARIES}
  libntcan.so
  lla2map_converter
)
target_link_libraries(ins_visualizer
  ${catkin_LIBRARIES}
  lla2map_converter
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  can_decode
  ins_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

#add_dependencies(esdcan ${catkin_EXPORTED_TARGETS})
ament_export_include_directories(include)
ament_package()
