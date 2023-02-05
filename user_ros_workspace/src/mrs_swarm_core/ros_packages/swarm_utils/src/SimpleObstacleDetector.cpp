#include "nodes/SimpleObstacleDetector.h"
#include <eigen3/Eigen/Eigen>
#include <unordered_set>
#include <cmath>
#include <queue>
#include <stack>
#include <tuple>

/* Map //{ */

class Map {
  using c_t = int32_t;

  c_t w, h;
  Eigen::Affine3d transform;
  Eigen::Affine3d inverseTransform;
  std::vector<int8_t> dat;
  double r;

public:
  Map(c_t w, c_t h, double resolution, const Eigen::Affine3d& transform):
    w(w), h(h), dat(w * h), r(resolution), transform(transform), inverseTransform(transform.inverse()) 
  {}

  c_t width() { return w; }
  c_t height() { return h; }
  double resolution() { return r; }

  int8_t& operator()(c_t x, c_t y) {
    return dat[w * y + x];
  }

  Map(const boost::shared_ptr<const nav_msgs::OccupancyGrid> msg) {
    w = msg->info.width;
    h = msg->info.height;
    dat = msg->data;
    r = msg->info.resolution;
    inverseTransform = Eigen::Affine3d::Identity();
    inverseTransform = inverseTransform
      .translate(Eigen::Vector3d(msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z))
      .rotate(Eigen::Quaterniond(msg->info.origin.orientation.w, msg->info.origin.orientation.x,
            msg->info.origin.orientation.y, msg->info.origin.orientation.z))
      .scale(msg->info.resolution);

    transform = inverseTransform;
    transform = transform.inverse();
  }
  
  Eigen::Vector3d toPosition(c_t x, c_t y) {
    return inverseTransform * Eigen::Vector3d(x, y, 0);
  }

  Eigen::Vector3d toPosition(double x, double y) {
    return inverseTransform * Eigen::Vector3d(x, y, 0);
  }

  std::pair<c_t,c_t> toCoordinates(const Eigen::Vector3d& position) {
    Eigen::Vector3d value = transform * position;
    return std::pair<c_t, c_t>(std::round(value(0)), std::round(value(1))); 
  }

  Map getSubMap(const Eigen::Vector3d& pos, double size) {
    auto c = toCoordinates(pos);
    auto cStart = std::make_pair<c_t, c_t>(std::round(c.first - size / r), std::round(c.second - size / r));
    auto cEnd = std::make_pair<c_t, c_t>(std::round(c.first + size / r), std::round(c.second + size / r));

    if (cStart.first < 0) cStart.first = 0;
    if (cStart.second < 0) cStart.second = 0;
    if (cEnd.first >= w) cEnd.first = w - 1;
    if (cEnd.second >= h) cEnd.second = h - 1;

    if (cEnd.first < cStart.first || cEnd.second < cStart.second)
      return Map(0, 0, r, transform);

    Eigen::Affine3d newTransform = transform;
    newTransform.pretranslate(-Eigen::Vector3d(cStart.first, cStart.second, 0));

    Map subMap = Map(
        cEnd.first - cStart.first, 
        cEnd.second - cStart.second, 
        r,
        newTransform);

    for (c_t y = 0; y < subMap.h; ++y) {
      for (c_t x = 0; x < subMap.w; ++x) {
        subMap(x, y) = operator()(cStart.first + x, cStart.second + y);
      }
    }

    return subMap;
  }

  struct CoordHash {
    inline size_t operator()(const std::pair<c_t, c_t>& c) const {
      return c.first * c.second;
    };
  };

};


//}

namespace swarm_utils {

  namespace nodes {

    /* simpleObstacleDetector() //{ */

    std::vector<geometry_msgs::Point> simpleObstacleDetector
      (std::shared_ptr<mrs_lib::Transformer> transformer, 
       boost::shared_ptr<const nav_msgs::OccupancyGrid> grid, 
       geometry_msgs::PoseStamped pose,
       double max_distance,
       double obstacle_size)
      {
        Eigen::Vector3d position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        std::vector<geometry_msgs::Point> obstaclePoints;

        // Transform position to map frame
        auto droneToMapTransform = 
          transformer->getTransform(pose.header.frame_id, grid->header.frame_id);
        if (!droneToMapTransform.has_value()) {
          ROS_ERROR_STREAM("[SimpleObstacleDetector] No transform between position frame " << 
              pose.header.frame_id << " and map frame " << grid->header.frame_id);
          return obstaclePoints;
        }
        Eigen::Affine3d droneToMap = droneToMapTransform.value().transformtoEigen();
        Eigen::Affine3d mapToDrone = droneToMap.inverse();

        // Create subMap
        Map map(grid);
        Map mapPart = map.getSubMap(
            position, 
            max_distance);

        // Find the obstacles
        std::vector<Eigen::Vector3d> obstacles;

        int maxDistance = std::ceil(obstacle_size / mapPart.resolution());
        std::unordered_set<std::pair<int32_t, int32_t>, Map::CoordHash> usedCoordinates;

        for (int32_t y = 0; y < mapPart.height(); ++y) {
          for (int32_t x = 0; x < mapPart.width(); ++x) {
            if (mapPart(x, y) <= 0) continue;
            if (usedCoordinates.count(std::make_pair(x, y))) continue;

            double centerX = 0;
            double centerY = 0;
            int number = 0;

            using stack_t = std::tuple<int32_t, int32_t, int>;
            std::queue<stack_t> queue;
            queue.emplace(x, y, 0);

            while (!queue.empty()) {
              stack_t top = queue.front();
              queue.pop();
              int32_t cx = std::get<0>(top);
              int32_t cy = std::get<1>(top);

              if (usedCoordinates.count(std::make_pair(cx, cy))) 
                continue;
              usedCoordinates.emplace(cx, cy);

              centerX += cx;
              centerY += cy;
              ++number;

              if (std::get<2>(top) >= maxDistance) continue;
              if (cx > 0 && !usedCoordinates.count(std::make_pair(cx - 1, cy)) && mapPart(cx - 1, cy) > 0) 
                queue.emplace(cx - 1, cy, std::get<2>(top) + 1);
              if (cx < mapPart.width() && !usedCoordinates.count(std::make_pair(cx + 1, cy)) && mapPart(cx + 1, cy) > 0)
                queue.emplace(cx + 1, cy, std::get<2>(top) + 1);
              if (cy > 0 && !usedCoordinates.count(std::make_pair(cx, cy - 1)) && mapPart(cx , cy - 1) > 0) 
                queue.emplace(cx, cy - 1, std::get<2>(top) + 1);
              if (cy < mapPart.height() && !usedCoordinates.count(std::make_pair(cx , cy + 1)) && mapPart(cx, cy + 1) > 0) 
                queue.emplace(cx, cy + 1, std::get<2>(top) + 1);
            }

            centerX /= number;
            centerY /= number;

            obstacles.emplace_back(mapPart.toPosition(centerX, centerY));
          }
        }

        for (size_t i = 0; i < obstacles.size(); ++i) {
          Eigen::Vector3d obstacle = mapToDrone * obstacles[i];

          geometry_msgs::Point point;
          point.x = obstacle(0);
          point.y = obstacle(1);
          point.z = obstacle(2);
          obstaclePoints.push_back(point);
        }

        return obstaclePoints;
      }

    //}

  }
}
