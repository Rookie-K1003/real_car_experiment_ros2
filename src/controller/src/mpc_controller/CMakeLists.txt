cmake_minimum_required(VERSION 3.21)
project(mpc_controller)

# 设置依赖
set(DEPENDS 
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
  "tf2"
  "ecl_geometry"
)

# 添加动态库
add_library(mpc_controller SHARED 
  ${CMAKE_CURRENT_SOURCE_DIR}/mpc_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/path_adapter.cpp
)

set(target 
  mpc_controller
)

# 添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target} 
  config_reader
  mathlibs
)

# 安装动态库、节点和相关目录
install(TARGETS 
  ${target}
  LIBRARY DESTINATION lib)