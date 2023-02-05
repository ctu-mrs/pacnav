#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mrs_lib/transformer.h>
#include <memory>

namespace swarm_utils {
      
  namespace nodes {

    // Takes the pose of a drone and occupancy grid of obstacles
    // Parameter max_distance - if obstacle is further than this distance - ignore it
    // Parameter obstacle_size - obstacles are aggregated to this size and not larger
    //
    // Returns the positions of obstacles in the frame of the drone pose

    std::vector<geometry_msgs::Point> simpleObstacleDetector
      (std::shared_ptr<mrs_lib::Transformer> transformer, 
       boost::shared_ptr<const nav_msgs::OccupancyGrid> grid, 
       geometry_msgs::PoseStamped pose,
       double max_distance=3,
       double obstacle_size=0.3);
  }
}
