#pragma once
#include <cstdlib>
#ifndef PACNAV_CONTROLLER_H
#define PACNAV_CONTROLLER_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>
/* #include <multi_uav_dynreconfig/DynReconfigConfig.h> */

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for mathematicla operations */
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <queue>
#include <map>
#include <random>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

/* custom msgs of MRS group */
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

/* for operations with matrices */
#include <eigen3/Eigen/Eigen>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>

/* from swarm_control_manager */
#include <swarm_control_manager/swarm_controller.h>
#include <swarm_control_manager/common_data_handler.h>

#include <swarm_utils/IdStateStamped.h>
#include <swarm_utils/IdStateArrayStamped.h>
#include <swarm_utils/ControlCommand.h>
#include <swarm_utils/ros_utils.h>
#include <swarm_utils/math_utils.h>

#include <pacnav/IdPointStamped.h>
//}

using namespace std;
namespace e = Eigen;
namespace swm_ctrl = swarm_control_manager;
namespace swm_utils  = swarm_utils;
namespace swm_r_utils  = swarm_utils::ros_utils;
namespace swm_m_utils  = swarm_utils::math_utils;

namespace pacnav 
{

  /* class PacnavController //{ */
  class PacnavController : public swm_ctrl::SwarmController{

    public:
      virtual void initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& ros_name_space, std::shared_ptr<mrs_lib::Transformer> transformer);
      virtual void activate();
      virtual void deactivate();
      virtual std::optional<std::any> update(std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data);

    private:
      /* flags */
      atomic<bool> is_init_                                = false;
      atomic<bool> is_active_ = false;

      /* ros parameters //{ */

      /* These params are laoded at init and SHOULD NOT BE CHANGED duing runtime */
      string _uav_name_ = "uav2";
      string _uav_frame_id_                        = "uav2/fcu";
      string _origin_frame_id_                     = "uav2/local_origin";
      atomic<double> _takeoff_height_                      = 2.0;
      atomic<double> _max_vel_                      = 0.0;
      atomic<double> _lidar_range_ = 10;
      atomic<double> _sigma_occl_ = 1.0;
      atomic<bool> _track_neighbors_ = false;

      atomic<double> _uav_radius_                          = 2;
      atomic<double> _flock_safety_rad_ = 4;
      atomic<double> _invalidate_time_ = 1;
      atomic<double> _path_invalidate_time_ = 1;
      atomic<double> _obst_f_const_                 = 1;
      atomic<double> _obst_f_max_dist_             = 3;
      atomic<double> _k_nav_                 = 1;

      //}

      // | ---------------------- common variables --------------------- |
      string _name_;
      bool use_lidar_ = false;
      bool use_map_ = false;

      std::shared_ptr<mrs_lib::Transformer> transformer_;

      pacnav::IdPointStamped target_;
      geometry_msgs::Vector3Stamped cur_des_vel_;
      map<int, geometry_msgs::PointStamped> neighbor_states_;
      map<int, ros::Time> los_stamps_;
      map<int, vector<geometry_msgs::PointStamped>> neighbor_paths_;

      ros::Publisher pub_mrs_uav_status_;

      // | ---------------------- callbacks --------------------- |

      /* void callbackPath(mrs_lib::SubscribeHandler<nav_msgs::Path>& sh); */
      mrs_lib::SubscribeHandler<nav_msgs::Path> sh_des_path_;
      /* atomic<bool>                                  got_path_ = false; */

      bool callbackSetGoal(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
      ros::ServiceServer srv_set_goal_;
      mutex mutex_goal_;
      geometry_msgs::PointStamped goal_;
      atomic<bool> got_goal_ = false;;

      // | --------------------- service clients -------------------- |

      ros::ServiceClient client_path_fndr_;

      // | -------------------- functions ------------------- |

      e::Vector2d calcObstForce(geometry_msgs::Vector3Stamped& prev_vel, swm_utils::IdStateArrayStamped& neighbor_states, sensor_msgs::LaserScanConstPtr lidar_data);
      ros::Publisher                             pub_viz_vir_obst_force_;
      ros::Publisher                             pub_viz_vir_obst_;
      ros::Publisher                             pub_viz_obst_force_;

      void updateTarget(pacnav::IdPointStamped& cur_target, swm_utils::IdStateStamped& self_state, map<int, vector<geometry_msgs::PointStamped>>& neighbor_paths);
      ros::Publisher                             pub_viz_target_;

      geometry_msgs::Vector3Stamped calcDesVel(geometry_msgs::Vector3Stamped& prev_vel, swm_utils::IdStateArrayStamped& neighbor_states, nav_msgs::Path& des_path, sensor_msgs::LaserScanConstPtr lidar_data);
      ros::Publisher                             pub_viz_nav_vec_;
      ros::Publisher                             pub_viz_des_vel_;

      double calcHeading(swm_utils::IdStateArrayStamped& neighbor_states);

      // | -------------------- utility functions ------------------- |

      /* | -------------------- vector utils ------------------- | //{*/

      void pubVecViz(geometry_msgs::Vector3Stamped& force, ros::Publisher& pub, std_msgs::ColorRGBA color);

      //}

      /* | -------------------- neighbor info utils ------------------- | //{*/

      void updateNeighborStates(map<int, geometry_msgs::PointStamped>& neighbor_states, swm_utils::IdStateArrayStampedConstPtr rec_neighbor_states, map<int, ros::Time>& los_stamps, sensor_msgs::LaserScanConstPtr lidar_data);
      ros::Publisher pub_viz_neighbor_states_;

      void updateNeighborPaths(map<int, vector<geometry_msgs::PointStamped>>& neighbor_paths, map<int, geometry_msgs::PointStamped>& neighbor_states);
      ros::Publisher pub_viz_neighbor_paths_;

      void applyOcllusion(map<int, geometry_msgs::PointStamped>& neighbor_states, swm_utils::IdStateArrayStampedConstPtr rec_neighbor_states, map<int, ros::Time>& los_stamps, vector<double> lidar_data);

      bool neighborInLOS(const std_msgs::Header& header, const geometry_msgs::Point& position, vector<double>& lidar_data);

      //}

      /* | -------------------- navigation force utils ------------------- | //{*/

      e::Vector2d getNextNavPoint(nav_msgs::Path& des_path);
      e::Vector2d rescaleNavVec(e::Vector2d& nav_vec, swm_utils::IdStateArrayStamped& neighbor_states, double flock_rad, bool got_goal);

      //}

  };
  //}

  // | -------------------- non-member functions ------------------- |

  /* | -------------------- target selection utils ------------------- | //{*/

  bool isCandConverging(geometry_msgs::Point& target, vector<geometry_msgs::PointStamped>& cand_path);
  map<int, map<int, float>> getSimilarityMat(map<int, vector<geometry_msgs::PointStamped>>& neighbor_paths);

  //}

  /* | -------------------- obst force utils ------------------- | //{*/

  int                                   angle2Index(double angle, int max_index);
  double                                index2Angle(int index, int max_index);
  vector<double> cleanLidarData(sensor_msgs::LaserScanConstPtr lidar_data, double max_range);
  e::Vector2d                           getClosestVec(e::Vector2d& comp_vec, e::Vector2d& base_vec, double des_mag, double des_dir);
  vector<e::Vector2d>                   getObstLidar(vector<double>& lidar_ranges, double max_range);

  //}

}  // namespace pacnav  

#endif
