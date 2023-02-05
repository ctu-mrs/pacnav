#pragma once
#ifndef BOIDS_CONTROLLER_H
#define BOIDS_CONTROLLER_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading dynamic parameters while the nodelet is running */
/* #include <dynamic_reconfigure/server.h> */
/* #include <multi_uav_dynreconfig/DynReconfigConfig.h> */

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for mathematica operations */
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

/* custom msgs of MRS group */
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/TransformVector3Srv.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/subscribe_handler.h>

/* for operations with matrices */
#include <eigen3/Eigen/Eigen>

/* from swarm_control_manager */
#include <swarm_control_manager/swarm_controller.h>
#include <swarm_control_manager/common_data_handler.h>

#include <swarm_utils/IdStateStamped.h>
#include <swarm_utils/IdStateArrayStamped.h>
#include <swarm_utils/ControlCommand.h>

#include <swarm_utils/ros_utils.h>

//}

using namespace std;
namespace e = Eigen;
namespace swm_ctrl = swarm_control_manager;
namespace swm_utils  = swarm_utils;
namespace swm_r_utils  = swarm_utils::ros_utils;

namespace boids 
{

  /* class BoidsController //{ */
  class BoidsController : public swm_ctrl::SwarmController {

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
      string _uav_name_                            = "uav2";
      string _uav_frame_id_                        = "uav2/fcu_untilted";
      string _origin_frame_id_                     = "uav2/local_origin";
      atomic<double> _takeoff_height_                      = 2.0;

      atomic<double> _uav_radius_                          = 2;
      atomic<double> _lidar_range_ = 10;
      atomic<double> _invalidate_time_ = 1;

      atomic<double> _prox_vec_zero_dist_ = 0.12;
      atomic<double> _prox_vec_crit_dist_ = 2.0;
      atomic<double> _prox_vec_rep_const_                = 4.0;

      atomic<double> _prox_vec_follow_dist_         = 0.2;
      atomic<double> _prox_vec_coh_const_ = 5.0;

      atomic<double> _col_vec_const_                  = 1;
      atomic<double> _col_vec_max_dist_              = 3;

      atomic<double> _k_prox_ = 1.0;
      atomic<double> _k_col_ = 1.0;
      atomic<double> _k_nav_= 1.0;

      atomic<double> _dir_vel_mag_ = 0.0;
      atomic<int> _dir_invalid_time_ = 1;
      //}

      // | ---------------------- common variables --------------------- |

      string _name_;
      std::shared_ptr<mrs_lib::Transformer> transformer_;
      ros::Publisher pub_mrs_uav_status_;
      map<std::string, int> dir_map_;
      mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped> sh_human_pose_;
      /* boost::recursive_mutex                                                                   mutex_dyn_reconfig_; */
      /* boost::shared_ptr<dynamic_reconfigure::Server<multi_uav_dynreconfig::DynReconfigConfig>> reconfig_srv_; */

      // | ---------------------- callbacks --------------------- |

      void                                  callbackPath(nav_msgs::PathConstPtr msg);
      ros::Subscriber                            sub_des_path_;
      nav_msgs::PathPtr des_path_;

      void callbackDesDirection(std_msgs::StringConstPtr msg);
      ros::Subscriber sub_des_dir_;
      atomic<int> des_dir_;
      ros::Time dir_cmd_time_;
      // | ------------------- dynamic reconfigure ------------------ |

      /* void                                  callbackDynReconfig([[maybe_unused]] multi_uav_dynreconfig::DynReconfigConfig& config, [[maybe_unused]] uint32_t level); */

      // | -------------------- utility functions ------------------- |

      /* | -------------------- vector utils ------------------- | //{*/

      void pubVecViz(geometry_msgs::Vector3Stamped ctrl_vec, ros::Publisher& pub, std_msgs::ColorRGBA color);

      //}

      /* | -------------------- collision control utils ------------------- | //{*/

      int                                   angle2Index(double angle, int max_index);
      double                                index2Angle(int index, int max_index);
      vector<double>                        cleanLidarData(sensor_msgs::LaserScanConstPtr lidar_data, double max_range);
      e::Vector2d getSteerVec(e::Vector2d comp_vec, e::Vector2d base_vec, double des_mag, double des_dir);
      vector<e::Vector2d> getObstFromLidar(vector<double> lidar_ranges, double max_range);

      //}

      /* | -------------------- nav control utils ------------------- | //{*/

      std::optional<geometry_msgs::PointStamped> getNextNavPoint(swm_utils::IdStateStampedConstPtr self_state, nav_msgs::PathConstPtr des_path);
      e::Vector2d rescaleNavCtrlVec(const e::Vector2d nav_vec, swm_utils::IdStateArrayStampedConstPtr neighbor_states);

      //}

      // | -------------------- functions ------------------- |

      geometry_msgs::Vector3Stamped calcProximalCtrlVec(swm_utils::IdStateArrayStampedConstPtr neighbor_states, string type);
      ros::Publisher                             pub_viz_proximal_vec_;

      geometry_msgs::Vector3Stamped calcCollisionCtrlVec(const geometry_msgs::Vector3Stamped pref_direction, sensor_msgs::LaserScanConstPtr lidar_data);
      ros::Publisher                             pub_viz_collision_vec_;
      ros::Publisher                             pub_viz_col_vec_list_;
      ros::Publisher                             pub_viz_vir_obst_;

      geometry_msgs::Vector3Stamped calcNavCtrlVec(swm_utils::IdStateStampedConstPtr self_state, swm_utils::IdStateArrayStampedConstPtr neighbor_states, nav_msgs::PathConstPtr des_path);
      ros::Publisher                             pub_viz_nav_vec_;

      geometry_msgs::Vector3Stamped calcDirCtrlVec(int des_dir, double vec_mag);
      ros::Publisher                             pub_viz_dir_vec_;

      std::optional<geometry_msgs::Vector3Stamped> calcFinalControlVec(const geometry_msgs::Vector3Stamped prev_final_vec, int des_dir, std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data);
      ros::Publisher                             pub_viz_final_vec_;
      geometry_msgs::Vector3Stamped final_vec_;

      double calcHeading(string req_frame_id, swm_utils::IdStateArrayStampedConstPtr neighbor_states);

  };
  //}

}  // namespace boids  

#endif
