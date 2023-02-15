cmake_minimum_required(VERSION 2.8.3)
project(pacnav)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nodelet
  cmake_modules
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
  swarm_control_manager
  swarm_utils
)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

add_message_files(DIRECTORY msg/target FILES

  IdPointStamped.msg
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
  LIBRARIES PacnavController 
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs message_runtime swarm_control_manager
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
add_library(PacnavController src/pacnav_controller.cpp)

add_dependencies(PacnavController
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## Specify libraries to link a library or executable target against
target_link_libraries(PacnavController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

# test cases
add_executable(PacnavTest test/pacnav_test.cpp)
target_link_libraries(PacnavTest 
  PacnavController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

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

# install(DIRECTORY doc
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
