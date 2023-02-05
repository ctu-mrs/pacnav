#include "path_finder/ros_message_convertors.h"

/* createGridMessage() //{ */

nav_msgs::OccupancyGrid createGridMessage(ObstacleGrid grid, std::string frame_id, Eigen::Vector3d centerPosition) {
  nav_msgs::OccupancyGrid message;

  message.data = grid.getData();
  message.info.width = grid.getWidth();
  message.info.height = grid.getHeight();
  message.info.resolution = grid.getResolution();
  message.info.origin.position.x = 
    centerPosition(0) - grid.getResolution() * grid.getWidth() / 2;
  message.info.origin.position.y = 
    centerPosition(1) - grid.getResolution() * grid.getHeight() / 2;
  message.info.origin.position.z = 0;
  message.header.frame_id = frame_id;
  message.header.stamp = ros::Time::now();

  return message;
}

nav_msgs::OccupancyGrid createGridMessage(ObstacleGrid grid, std::string frame_id) {
  return createGridMessage(grid, frame_id, Eigen::Vector3d{0, 0, 0});
}


//}

/* createPathMessage() //{ */

nav_msgs::Path createPathMessage(const Eigen::MatrixXd& matrix, Eigen::Vector3d pathSource, std::string frame_id) 
{
    nav_msgs::Path res;

    std_msgs::Header header;
    /* const double altitude = pathSource(2); */
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    res.header = header;
    res.poses = std::vector<geometry_msgs::PoseStamped>{static_cast<size_t>(matrix.rows())};

    for (int i = 0; i < matrix.rows(); ++i) {
      res.poses[i].header = header;
      res.poses[i].pose.position.x = matrix(i, 0);
      res.poses[i].pose.position.y = matrix(i, 1);
      res.poses[i].pose.position.z = 0;
    }
  return res;
}

//}

