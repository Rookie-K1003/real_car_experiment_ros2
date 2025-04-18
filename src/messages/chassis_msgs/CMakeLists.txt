cmake_minimum_required(VERSION 3.5.0)
project(chassis_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

if(POLICY CMP0057)
cmake_policy(SET CMP0057 NEW)
endif()

#SET(CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EHB.msg"
  "msg/EPS.msg"
  "msg/VCU.msg"
  "msg/ESC.msg"
  "msg/ChassisInfo.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()


# # ROS1
# find_package(catkin REQUIRED COMPONENTS  
#   message_generation
#   roscpp
# )


# add_message_files(
#   FILES
#   EHB.msg
#   EPS.msg
#   VCU.msg
#   ChassisInfo.msg
# )


# generate_messages(DEPENDENCIES
#   std_msgs
# )


# catkin_package(
#   CATKIN_DEPENDS message_runtime
# )

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )
