cmake_minimum_required(VERSION 2.8.3)
project(swarm_control_manager)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  cmake_modules
  nodelet
  mrs_lib
  nav_msgs
  mrs_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
  message_generation
  swarm_utils
)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SwarmControlManager TestController 
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs std_msgs geometry_msgs nav_msgs
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
add_library(SwarmControlManager src/swarm_control_manager.cpp)
add_library(TestController src/test_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(SwarmControlManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

target_link_libraries(TestController 
  SwarmControlManager
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

# target_link_libraries(
#   CommonTfPublisher
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
# )

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
