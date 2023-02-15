#pragma once
#ifndef SWARM_CONTROL_MANAGER_H
#define SWARM_CONTROL_MANAGER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <mrs_msgs/Path.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <atomic>
#include <any>
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

#include <swarm_control_manager/swarm_controller.h>
#include <swarm_control_manager/common_data_handler.h>
#include <swarm_utils/ros_utils.h>

#include <swarm_utils/IdStateStamped.h>
#include <swarm_utils/IdStateArrayStamped.h>


using namespace std;
namespace swm_ctrl = swarm_control_manager;
namespace swm_utils = swarm_utils;
namespace swm_r_utils = swarm_utils::ros_utils;

namespace swarm_control_manager
{

// | ------------------------ Utility struct ---------------------- |
  
struct SwarmControlManagerState {

  bool is_active = false;
  bool tracker_ready = false;
};

struct LoadedController {

  string address;
  string ros_name_space;
  map<string, vector<string>> required_data;
  string neighbor_localization;
  string control_type;

  boost::shared_ptr<swm_ctrl::SwarmController> plugin;
};

// | ------------------------ plugin class ------------------------ |

class SwarmControlManager: public nodelet::Nodelet {

  public:
  virtual void onInit();

  private:

  // | ------------------------ common variables ------------------ |
  
  atomic<bool> is_initialized_ = false;
  mutex mutex_manager_state_;
  SwarmControlManagerState manager_state_;

  mutex mutex_neighbor_states_;
  swm_utils::IdStateArrayStampedPtr neighbor_states_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;
  std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data_;

  mutex mutex_current_controller_;
  string current_controller_;

  std::unique_ptr<pluginlib::ClassLoader<swm_ctrl::SwarmController>> controller_loader_;
  map<string, LoadedController> loaded_controllers_;

  /* ros parameters //{ */

  string _uav_name_;
  string _uav_frame_id_;
  string _origin_frame_id_;

  map<string, vector<string>> _allowed_data_;
  vector<string> _neighbor_localization_types_;
  vector<string> _control_types_;

  atomic<double> _control_timer_rate_;
  atomic<double> _invalidate_time_;
  atomic<double> _traj_gen_time_;
  atomic<double> _vel_max_;

  //}

  // | ------------------------ topic callbacks ---------------------|

  void callbackOdometry(const mrs_msgs::PositionCommandConstPtr& msg);
  ros::Subscriber sub_odom_;
  mutex mutex_self_state_;
  swm_utils::IdStateStampedPtr self_state_;

  void callbackHectorMap(const nav_msgs::OccupancyGridConstPtr& msg);
  ros::Subscriber sub_hector_map_;
  mutex mutex_self_hector_map_;
  nav_msgs::OccupancyGridPtr self_hector_map_;

  void callbackLaserScan(const sensor_msgs::LaserScanConstPtr& msg);
  ros::Subscriber sub_laser_scan_;
  mutex mutex_self_laser_scan_;
  sensor_msgs::LaserScanPtr self_laser_scan_;

  void callbackUvdarFilteredStates(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& msg);
  ros::Subscriber sub_uvdar_filtered_;
  atomic<bool> got_uvdar_cb_ = false;

  void callbackSharedGps(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& msg);
  ros::Subscriber sub_shared_gps_;
  atomic<bool> got_shared_gps_cb_ = false;

  // | ------------------------ publishers ------------------------ |
  
  ros::Publisher pub_vel_ref_;

  // | ------------------------ services -------------------------- |
  
  ros::ServiceServer server_run_swarm_controller_;
  bool callbackRunSwarmController([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  
  ros::ServiceServer server_activate_swarm_controller_;
  bool callbackActivateSwarmController(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  // | ------------------------ service clients ------------------- |

  ros::ServiceClient client_switch_tracker_;
  ros::ServiceClient client_path_ref_;

  // | ------------------------ timers ---------------------------- |
  
  void callbackControlTimer(const ros::TimerEvent &);
  ros::Timer timer_control_;

  // | ------------------------ routines -------------------------- |

  bool loadController(std::string &controller_name, std::string &control_type);
  bool isManagerReady(std::string &current_controller);
  bool switchTracker(const string tracker_name);

  // | ---------------------- getter routines --------------------- |

  swm_utils::IdStateStampedConstPtr getSelfState();
  nav_msgs::OccupancyGridConstPtr getSelfHectorMap();
  sensor_msgs::LaserScanConstPtr getSelfLaserScan();
  swm_utils::IdStateArrayStampedConstPtr getNeighborStates();

};

}

#endif
