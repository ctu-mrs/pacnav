/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <stdint.h>
#include <cmath>
#include <sstream>
#include <string>
#include <mutex>
#include <eigen3/Eigen/Eigen>

#include <nav_msgs/OccupancyGrid.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/PositionCommand.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

/*for image processing*/
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <mrs_lib/param_loader.h>

#include "path_finder/grid.h"
#include "spline/spline.h"
#include "path_finder/ros_message_convertors.h"
#include "pathfindingGeneral/pathfindingGeneral.h"
#include "uvdar_listener.h"

//}

/* defines //{ */

#define DEBUG
// #define FLIGHT

ros::Subscriber    map_subscriber;
ros::Subscriber    pose_subscriber;
ros::Subscriber    odom_main_subscriber;
ros::Subscriber    position_cmd_subscriber;

std::shared_ptr<UvdarListener> uvdarListener;
bool use_uvdar = false;
double uvdar_uav_size = 1;

std::shared_ptr<mrs_lib::Transformer> transformer_;

#ifdef DEBUG
ros::Publisher     distance_map_publisher;
ros::Publisher     path_map_publisher;
#endif

ros::Publisher     path_publisher;
ros::Publisher     pub_clean_map;

ros::ServiceServer path_goto;
bool goto_flag = false;
std::mutex mutex_path_goto_;

ros::ServiceClient trajectory_client;

Eigen::Vector3d pathSource_;
Eigen::Vector3d pathTarget_;
std::mutex mutex_target_;
std::mutex mutex_pose_;

std::string _origin_frame_id_;
float required_resolution;
float min_safe_distance;
float max_caution_distance;
float trajectory_velocity;
bool use_closest_start_coord;
bool use_closest_finish_coord;

bool publish_local_map;
float local_map_distance;

// UvdarListener *uvdarListener;

//}

/* helpers //{ */
bool isPointinsideGridBorder(ObstacleGrid& grid, Eigen::Vector3d vector, double border) {
	Coordinate coordinate = grid.translateToCoordinates(vector);
	double gridBorder = border / grid.getResolution();
	if (coordinate.x > grid.getWidth() - gridBorder || coordinate.y > grid.getHeight() - gridBorder) {
	  ROS_WARN_THROTTLE(15,"Target outside map limits");
    return false;
  }

  /* vector = grid.translateFromCoordinates(coordinate, vector(2)); */
	/* return vector; */
  return true;
}

std::vector<int8_t> cleanMapData(const std::vector<int8_t> map_data, const uint32_t map_height, const uint32_t map_width){
  std::vector<int8_t> clean_map_data(map_data.size());
  cv::Mat map_data_img;
  map_data_img = cv::Mat::zeros(map_width, map_height, CV_32F);

  for(size_t y = 0; y < map_height; y++) {
    for(size_t x = 0; x < map_width; x++) {
      map_data_img.at<float>(x,y) = map_data[y * map_width + x] <= 0 ? 0.0 : 1.0;
    }
  }

  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  dilate(map_data_img, map_data_img, element, cv::Point(-1, -1), 4); 
  element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  erode(map_data_img, map_data_img, element, cv::Point(-1, -1), 2); 

  for(size_t y = 0; y < map_height; y++) {
    for(size_t x = 0; x < map_width; x++) {
      clean_map_data[y * map_width + x] = 100 * map_data_img.at<float>(x,y);
    }
  }
    
  return clean_map_data;
}
//}

/* subscriber mapCallback() //{ */

void mapCallback(const nav_msgs::OccupancyGrid& msg) {

  /* std::cout<<"In am in map calllback"<<std::endl; */
  Eigen::Vector3d pathSource, pathTarget;
  {
    std::scoped_lock lock(mutex_target_);
    pathSource = pathSource_;
    pathTarget = pathTarget_;
  }

  std::vector<int8_t> cleaned_map = cleanMapData(msg.data, msg.info.height, msg.info.width);

  nav_msgs::OccupancyGrid msg_clean_map = msg;
  msg_clean_map.data = cleaned_map;
  pub_clean_map.publish(msg_clean_map);

  ObstacleGrid grid{msg.info.width, msg.info.height, msg.info.resolution, cleaned_map};
  uint32_t     requiredScale = std::ceil(required_resolution / grid.getResolution()); 
  auto         scaledGrid = scaleResolution(grid, requiredScale);

  std::vector<Eigen::Vector3d> uavStates = uvdarListener->getEigenPositions();
  for (Eigen::Vector3d uavState: uavStates) {
    scaledGrid.drawObstacleCircle(uavState, uvdar_uav_size);
  }

  auto voronoi = scaledGrid.createVoronoiGraph(min_safe_distance / scaledGrid.getResolution(), max_caution_distance / scaledGrid.getResolution());
  auto obstacleGridScaled = voronoi.map<int8_t>([](bool value) { return value ? 100 : 0; });

  if (publish_local_map) {
    auto obstacleGridPart = obstacleGridScaled.getSubGridAtPosition(pathSource, local_map_distance);
    distance_map_publisher.publish(createGridMessage(obstacleGridPart, _origin_frame_id_, pathSource));
  } else {
    distance_map_publisher.publish(createGridMessage(obstacleGridScaled, _origin_frame_id_));
  }

  if (goto_flag) {
    PathFindingResult result = findPath(voronoi, 
        voronoi.translateToCoordinates(pathSource), 
        voronoi.translateToCoordinates(pathTarget));

    if (result.path.empty()){
      ROS_WARN("[Path Finder] No Path found for given start and end");
    } else if (result.startModified && !use_closest_start_coord) {
      ROS_WARN("[Path Finder] "
          "No Path was found because the start coordinate is invalid");
    } else if (result.finishModified && !use_closest_finish_coord) {
      ROS_WARN("[Path Finder] "
          "No Path could be found. The closest possible can be generated "
          "if you set the use_closest_finish_coord parameter");
    } else {
      std::vector<Coordinate> path = result.path;
      ObstacleGrid pathGrid{
        scaledGrid.getWidth(), 
        scaledGrid.getHeight(), 
        scaledGrid.getResolution(), 0};

      for (Coordinate coord: path) {
        pathGrid(coord) = 101;
      }

      path_map_publisher.publish(createGridMessage(pathGrid, _origin_frame_id_));

      // create points
      int cellPerPathPoint = std::ceil(trajectory_velocity / 
          voronoi.getResolution() / 2);
      int pathPointsToOptimise = 10;
      size_t pathPoints = std::ceil((float) (path.size() - 1) / 
          pathPointsToOptimise / cellPerPathPoint) + 1;
      Eigen::MatrixXd points{pathPoints, 2};
      for (size_t i = 0; i < pathPoints; ++i) {
        size_t pathPoint = i * pathPointsToOptimise * cellPerPathPoint;
        if (pathPoint >= path.size()) pathPoint = path.size() - 1;
        points.row(i) = voronoi.translateFromCoordinates(path[pathPoint], pathSource(2));
      }

      // make spline of the points
      Eigen::MatrixXd splinePoints;
      if (points.rows() > 1) {
        splinePoints = buildCubicSplinePath(points, pathPointsToOptimise);
      } else {
        splinePoints = Eigen::MatrixXd{2, 2};
        splinePoints.row(0) = Eigen::RowVector2d{pathSource(0), pathSource(1)};
        splinePoints.row(1) = Eigen::RowVector2d{pathSource(0), pathSource(1)};
      }

      // publish pass
      path_publisher.publish(createPathMessage(splinePoints, pathSource, _origin_frame_id_));
      ROS_INFO("[Path Finder] Publishing path");
    }
  } 
  ROS_DEBUG_THROTTLE(15, "[Path Finder]: Map received");
}

//}

/* callbackOdometry() //{ */

void callbackOdometry(const mrs_msgs::PositionCommand& msg) {
  mrs_msgs::ReferenceStamped tmp;
  tmp.header = msg.header;
  tmp.reference.position = msg.position;
  tmp.reference.heading = 0.0;
  auto tf_pose = transformer_->transformSingle(tmp, _origin_frame_id_);
  if(tf_pose) {
    std::scoped_lock lock(mutex_pose_);
    pathSource_ = Eigen::Vector3d{tf_pose.value().reference.position.x, tf_pose.value().reference.position.y, tf_pose.value().reference.position.z};
  }
  else {
    ROS_ERROR("[Path Finder]: Tf 'odom' failed. From %s to %s", msg.header.frame_id.c_str(), _origin_frame_id_.c_str());
    return ;
  }
}

//}

/* service pathGotoCallback() //{ */

bool pathGotoCallback(mrs_msgs::Vec4::Request& request, mrs_msgs::Vec4::Response& response) {
  Eigen::Vector3d source{0, 0, 0}, target{0, 0, 0};
  {
    std::scoped_lock lock(mutex_path_goto_);
  pathTarget_ = Eigen::Vector3d{request.goal[0], request.goal[1], request.goal[2]};
  source = pathSource_;
  target = pathTarget_;
  }

  std::stringstream pathss;
  pathss << "[" << source(0) << ", " << source(1) << ", " << source(2) << "]";
  pathss << " -> ";
  pathss << "[" << target(0) << ", " << target(1) << ", " << source(2) << "] (z same)";

  ROS_INFO("[Path Finder] Received Path GOTO request: %s", pathss.str().c_str());

  response.success = true;
  response.message = "Successfully set path: " + pathss.str() +  ".";
  goto_flag = true;
  return true;
}

//}

/* main() //{ */

int main(int argc, char** argv) {
  // initialize node and create node handle
  ros::init(argc, argv, "path_finder");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  std::string uav_name;

  // PARAMETERS
  mrs_lib::ParamLoader pl{nh_, "Path Finder"};
  pl.loadParam("uav_name", uav_name);
  pl.loadParam("origin_frame_id", _origin_frame_id_);
  pl.loadParam("required_resolution", required_resolution, (float) 0.0625);
  pl.loadParam("min_safe_distance", min_safe_distance);
  pl.loadParam("max_caution_distance", max_caution_distance);
  pl.loadParam("trajectory_velocity", trajectory_velocity, (float) 0.5);
  pl.loadParam("use_closest_start_coord", use_closest_start_coord, true);
  pl.loadParam("use_closest_finish_coord", use_closest_finish_coord, true);

  pl.loadParam("publish_local_map", publish_local_map, false);
  pl.loadParam("local_map_distance", local_map_distance, (float) 1.0);

  pl.loadParam("use_uvdar", use_uvdar, use_uvdar);
  pl.loadParam("uvdar_uav_size", uvdar_uav_size, uvdar_uav_size);

  transformer_.reset(new mrs_lib::Transformer(std::string("PathFinder")));

  // Uvdar Listener
  if (use_uvdar) {
    uvdarListener.reset(new UvdarListener(transformer_, _origin_frame_id_));
  } else {
    uvdarListener.reset(new UvdarListener());
  }
 
  // SUBSCRIBERS
  map_subscriber = nh_.subscribe("hector_map_in", 1, &mapCallback);
  position_cmd_subscriber = nh_.subscribe("position_command_in", 1, &callbackOdometry);

  // PUBLISHERS
#ifdef DEBUG
  distance_map_publisher = nh_.advertise<nav_msgs::OccupancyGrid>("distance_map", 500);
  path_map_publisher = nh_.advertise<nav_msgs::OccupancyGrid>("path_map", 500);
#endif

  path_publisher = nh_.advertise<nav_msgs::Path>("path", 500);
  pub_clean_map = nh_.advertise<nav_msgs::OccupancyGrid>("clean_map", 500);
  // SERVICE SERVERS
  path_goto = nh_.advertiseService("path_goto_in", &pathGotoCallback);

  ROS_INFO("[Path Finder] Node Initialized");

  if (ros::ok()) {
    ros::spin();
    return 0;
  }
}

//}

