cmake_minimum_required(VERSION 3.5)
project(esdcan)

# add_compile_options(-std=c++14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # 20250416
endif()
message("CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(location_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(chassis_msgs REQUIRED) # 20250416
find_package(visualization_msgs REQUIRED)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)




set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CanCommonFcn.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CanHandler.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CanMsgDefs.cc
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CanCommonFcn.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CanHandler.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CanMsgDefs.h
    #${CMAKE_CURRENT_SOURCE_DIR}/include/ntcan.h	
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

#link_directories(${catkin_LIB_DIRS} dll)
link_directories(${catkin_LIB_DIRS})
add_library(canlib src/CanHandler.cc src/CanMsgDefs.cc src/CanCommonFcn.cc)

# Node
# add_executable(esdcan src/main.cc ${HEADERS} ${SOURCES}) # 包含完整的与底盘CAN通信的功能
add_executable(esdcan src/main_recv.cc ${HEADERS} ${SOURCES}) # 仅进行CAN接收，不发送，用于数据采集 20250418

ament_target_dependencies(esdcan
    rclcpp
    std_msgs
    geometry_msgs
    location_msgs
    control_msgs
    chassis_msgs # 20250416
    visualization_msgs # 20250418
)
#target_link_libraries(esdcan ${CMAKE_CURRENT_SOURCE_DIR}/dll/libCanCmd.so ${catkin_LIBRARIES} pthread glog)
target_link_libraries(esdcan ${catkin_LIBRARIES} libntcan.so pthread glog)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  esdcan
  DESTINATION lib/${PROJECT_NAME}
)

#add_dependencies(esdcan ${catkin_EXPORTED_TARGETS})
ament_export_include_directories(include)
ament_package()







