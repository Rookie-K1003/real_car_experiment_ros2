cmake_minimum_required(VERSION 3.5)
project(data_manager)

# 使用 C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 添加可执行文件
add_executable(pointcloud_time_printer src/timestamp_test/pointcloud_time_printer.cpp)

# 链接依赖
ament_target_dependencies(pointcloud_time_printer
  rclcpp
  sensor_msgs
)

# 安装
install(TARGETS
  pointcloud_time_printer
  DESTINATION lib/${PROJECT_NAME}
)

# 结束
ament_package()
