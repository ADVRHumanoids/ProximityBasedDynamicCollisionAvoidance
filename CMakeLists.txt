cmake_minimum_required(VERSION 3.5)
project(dyn_collision_avoid)

# c++ compiler
if(NOT CMAKE_CXX_STANDARD) #Default to C++14
  set(CMAKE_CXX_STANDARD 14)
endif()

# ros2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# custom messages
find_package(rosidl_default_generators REQUIRED)

# external
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(matlogger2 REQUIRED)

# to generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SensorsMsg.msg"
    "msg/RobotMsg.msg"
    "msg/MotionMsg.msg"
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(Logger
    utils/logger.cpp
)

add_library(Algebra
    utils/algebra.cpp
)

add_library(Utilities
    utils/utilities.cpp
)

add_library(KDLHelper
    utils/KDLhelper.cpp
)

add_library(DynamicCollisionAvoidanceManager
 src/DynamicCollisionAvoidanceManager.cpp
)

set(dependencies
        "rclcpp")

######################################################################################################
add_executable(DynamicCollisionAvoidance src/DynamicCollisionAvoidance.cpp)
ament_target_dependencies(DynamicCollisionAvoidance ${dependencies})
ament_export_dependencies(DynamicCollisionAvoidance message_runtime)


target_link_libraries(Logger
   matlogger2::matlogger2
)

target_link_libraries(Utilities
  Algebra
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)


target_link_libraries(DynamicCollisionAvoidanceManager
  "${cpp_typesupport_target}"
  KDLHelper
  Algebra
  Utilities
  Logger
)

target_link_libraries(DynamicCollisionAvoidance
   DynamicCollisionAvoidanceManager
)

ament_target_dependencies(KDLHelper kdl_parser)
ament_target_dependencies(DynamicCollisionAvoidanceManager ${dependencies})
ament_export_dependencies(rosidl_default_runtime)

# to allow to ros2 to find the executable
install(TARGETS
  DynamicCollisionAvoidance
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# To make config visible
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()