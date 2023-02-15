cmake_minimum_required(VERSION 2.8.3)
project(forest_path_finder CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  mrs_lib
  nav_msgs
  mrs_msgs
  geometry_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs
)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS mrs_lib nav_msgs mrs_msgs geometry_msgs
  DEPENDS Eigen
  LIBRARIES grid ros_message_convertors
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/path_finder
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(grid include/path_finder/grid.h)
set_target_properties(grid PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(grid ${Eigen_LIBRARIES})

add_library(aStar src/pathfindingGeneral/aStar/aStar.cpp)
target_link_libraries(aStar grid ${Eigen_LIBRARIES})

add_library(jumpPointSearch src/pathfindingGeneral/jumpPointSearch/jumpPointSearch.cpp)
target_link_libraries(jumpPointSearch grid ${Eigen_LIBRARIES})

add_library(spline src/spline/spline.cpp)
target_link_libraries(spline ${Eigen_LIBRARIES})

add_library(ros_message_convertors src/ros_message_convertors/ros_message_convertors.cpp)
target_link_libraries(ros_message_convertors ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_library(aStarNew INTERFACE)
target_link_libraries(aStarNew INTERFACE ${Eigen_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(aStarNew INTERFACE src/pathfindig/)

add_library(uvdar_listener src/uvdar_listener.cpp)
target_link_libraries(uvdar_listener ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_executable(forest_path_finder 
  src/forest_path_finder.cpp 
)
target_link_libraries(forest_path_finder
  grid
  aStar
  jumpPointSearch
  spline
  ros_message_convertors
  aStarNew
  uvdar_listener
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(TARGETS
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		PATTERN ".svn" EXCLUDE
)

install(DIRECTORY doc
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
