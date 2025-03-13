cmake_minimum_required(VERSION 3.21)
project(controller_process)

# 设置依赖
set(DEPENDS 
  "rclcpp"
  "nav_msgs"
  "base_msgs"
  "tf2"
  "tf2_ros"
)

# 添加节点
add_executable(controller_process
  ${CMAKE_CURRENT_SOURCE_DIR}/controller_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/controller_process.cpp
)

set(target 
  controller_process
)

# 添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  # basic_controller
  mpc_controller
)

# 安装动态库、节点和相关目录
install(TARGETS 
  ${target}
  DESTINATION lib/${CONTROLLER_PACKAGE_NAME})