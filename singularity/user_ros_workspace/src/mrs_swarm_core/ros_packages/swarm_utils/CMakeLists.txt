cmake_minimum_required(VERSION 2.8.3)
project(swarm_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  cmake_modules
  nodelet
  mrs_lib
  mrs_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  message_generation
)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

add_message_files(DIRECTORY msg FILES
  IdState.msg
  IdStateStamped.msg
  IdStateArrayStamped.msg
  ControlCommand.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RosUtils MathUtils SharedGpsAggr CommonTfPub #SimpleObstacleDetector
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs message_runtime 
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(SimpleObstacleDetector src/SimpleObstacleDetector.cpp)
add_library(RosUtils src/ros_utils.cpp)
target_link_libraries(RosUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(SharedGpsAggr src/shared_gps_aggr.cpp)
target_link_libraries(SharedGpsAggr
  RosUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

add_library(CommonTfPub src/common_tf_pub.cpp)
target_link_libraries(CommonTfPub
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# add_library(SimpleObstacleDetector src/SimpleObstacleDetector.cpp)
# target_link_libraries(
#   SimpleObstacleDetector
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
# )

add_library(MathUtils src/math_utils.cpp)
target_link_libraries(MathUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# test cases
add_executable(MathTest test/math_test.cpp)
target_link_libraries(MathTest
  MathUtils
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

catkin_install_python(PROGRAMS scripts/scheduler.py scripts/transform_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# target_link_libraries(
#   StateUpdater
#   Unscented
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
# )

# target_link_libraries(
#   DecAgentBase
#   StateUpdater
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
# )

# add_dependencies(TestAgent
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
#   )

# target_link_libraries(
#   SimpleObstacleDetector
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
# )

# install(TARGETS
# 		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
# 		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# 		FILES_MATCHING PATTERN "*.h"
# 		PATTERN ".svn" EXCLUDE
# )
