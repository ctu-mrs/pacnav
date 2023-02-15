#ifndef FOREST_PATH_FINDER_ROS_HELPER_FUNCTIONS_H
#define FOREST_PATH_FINDER_ROS_HELPER_FUNCTIONS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "path_finder/grid.h"
#include <eigen3/Eigen/Eigen>
#include <string>
#include <cmath>

nav_msgs::OccupancyGrid createGridMessage(ObstacleGrid grid, std::string frame_id, Eigen::Vector3d centerPosition);

nav_msgs::OccupancyGrid createGridMessage(ObstacleGrid grid, std::string frame_id);

nav_msgs::Path createPathMessage(const Eigen::MatrixXd& matrix, Eigen::Vector3d pathSource, std::string frame_id);

#endif //FOREST_PATH_FINDER_ROS_HELPER_FUNCTIONS_H
