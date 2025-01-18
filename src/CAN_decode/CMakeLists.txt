cmake_minimum_required(VERSION 3.5)
project(can_decode)

# add_compile_options(-std=c++14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# add_message_files(
#   FILES
#   rtk.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
)

# include_directories(
# 	include
# 	${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )

link_directories(${catkin_LIB_DIRS})

add_executable(can_decode
    src/can_decode_fix_bug.cpp)



#add_dependencies(can_decode ${catkin_EXPORTED_TARGETS})
ament_target_dependencies(can_decode
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    location_msgs
)
# add_executable(path_visu
#     src/path_visu.cpp)

# target_link_libraries(path_visu
#   ${catkin_LIBRARIES}
# )
target_link_libraries(can_decode
  ${catkin_LIBRARIES}
  libntcan.so
)



install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  can_decode
  DESTINATION lib/${PROJECT_NAME}
)

#add_dependencies(esdcan ${catkin_EXPORTED_TARGETS})
ament_export_include_directories(include)
ament_package()
